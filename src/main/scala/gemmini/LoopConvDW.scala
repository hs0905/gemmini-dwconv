package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.tile.RoCCCommand
import freechips.rocketchip.config.Parameters
import GemminiISA._
import LocalAddr._
import Util._



class LoopConvDWOuterBounds(val iterator_bitwidth: Int) extends Bundle {
  val in_dim = UInt(10.W)
  val num_channels = UInt(12.W)
  val out_dim = UInt(10.W)
  val stride = UInt(2.W)
  val kernel_dim = UInt(3.W)
  val ch_blk_num = UInt(9.W)
  val frame_size = UInt(8.W)
  val frame_height = UInt(6.W)
}

class LoopConvDWInnerBounds(val iterator_bitwidth: Int) extends Bundle {
  val krows = UInt(3.W)
  val kcols = UInt(3.W)
  val chs = UInt(12.W)
  val lpad = UInt(1.W)
  val rpad = UInt(1.W)
  val upad = UInt(1.W)
  val dpad = UInt(1.W)
  val orows = UInt(9.W)
  val ocols = UInt(9.W)
  val orows_ocols = UInt(18.W)
  val irows = UInt(9.W)
  val icols = UInt(9.W)
  val irows_icols = UInt(18.W)
  val atomic_orows = UInt(8.W)
  val atomic_irows_icols = UInt(17.W)
}

class LoopConvDWDerivedParams(val iterator_bitwidth: Int) extends Bundle {
  val irows_unpadded = UInt(iterator_bitwidth.W)
  val icols_unpadded = UInt(iterator_bitwidth.W)

  val bias_spad_stride = UInt(iterator_bitwidth.W)
  val input_spad_stride = UInt(iterator_bitwidth.W)
  val weight_spad_stride = UInt(iterator_bitwidth.W)

  // val ex_overwrite = Bool()
}

class LoopConvDWLdBiasReq(val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_acc_addr: Int, val concurrent_loops: Int)  extends Bundle {
  val outer_bounds = new LoopConvDWOuterBounds(iterator_bitwidth)
  val inner_bounds = new LoopConvDWInnerBounds(iterator_bitwidth)
  val derived_params = new LoopConvDWDerivedParams(iterator_bitwidth)
  val addr_start = UInt(log2Up(max_acc_addr).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val no_bias = Bool()
  val loop_id = UInt(log2Up(concurrent_loops).W)
}

class LoopConvDWLdBias(block_size: Int, coreMaxAddrBits: Int, val iterator_bitwidth: Int, max_acc_addr: Int, acc_w: Int,
                     max_block_len_acc: Int, concurrent_loops: Int, latency: Int,
                     config_mvin_rs1_t: ConfigMvinRs1, mvin_rs2_t: MvinRs2)(implicit p: Parameters) extends Module {
  val MVIN_SCALE_IDENTITY = 0x3f800000.U // TODO get this from configs somehow
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopConvDWLdBiasReq(coreMaxAddrBits, iterator_bitwidth : Int, max_acc_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())
    val wait_for_prev_loop = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, config, ld = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopConvDWLdBiasReq(coreMaxAddrBits, iterator_bitwidth: Int, max_acc_addr, concurrent_loops))
  import req.inner_bounds._
  import req.outer_bounds.ch_blk_num
  import req.derived_params._

  dontTouch(req)

  val acc_addr_start = req.addr_start

  val skip = req.dram_addr === 0.U

  // Iterators
  val cb = Reg(UInt(iterator_bitwidth.W))
  val offset = Reg(UInt(iterator_bitwidth.W))

  dontTouch(cb)
  dontTouch(offset)

  // Addresses
  val dram_offset = cb * block_size.U * (acc_w/8).U
  val dram_addr = Mux(req.no_bias, 0.U, req.dram_addr + LoopConvDW.castDramOffset(dram_offset))
  val spad_addr = acc_addr_start +& cb * orows_ocols +& offset

  // Sizes
  // val I = Mux(ocols - ocol > block_size.U, block_size.U, ocols - ocol)
  val I = Mux(orows_ocols - offset > block_size.U, block_size.U, orows_ocols - offset)
  val J = Mux(cb === ch_blk_num - 1.U, chs - block_size.U * cb, block_size.U)

  class RoCCCommandWithAddr extends Bundle {
    val cmd = new RoCCCommand
    val dram_addr = UInt()
    val spad_addr = UInt()
    val I = UInt()
    val J = UInt()
  }
  val command_p = Module(new Pipeline[RoCCCommandWithAddr](new RoCCCommandWithAddr, latency)())

  // Commands
  val config_cmd = Wire(new RoCCCommand)
  config_cmd := DontCare
  config_cmd.inst.funct := CONFIG_CMD

  val config_cmd_rs1 = Wire(config_mvin_rs1_t.cloneType)
  config_cmd_rs1 := DontCare
  config_cmd_rs1.scale := MVIN_SCALE_IDENTITY
  config_cmd_rs1.stride := req.derived_params.bias_spad_stride
  config_cmd_rs1.pixel_repeats := 1.U
  config_cmd_rs1.state_id := 2.U
  config_cmd_rs1.shrink := 0.U
  config_cmd_rs1._unused := 1.U
  config_cmd.rs1 := config_cmd_rs1.asUInt

  config_cmd.rs2 := 0.U

  val mvin_cmd = Wire(new RoCCCommand)
  mvin_cmd := DontCare
  mvin_cmd.inst.funct := LOAD3_CMD
  mvin_cmd.rs1 := 0.U
  mvin_cmd.rs2 := 0.U

  // Inputs and outputs
  io.req.ready := state === idle && !command_p.io.busy
  io.idle := state === idle && !command_p.io.busy
  io.loop_id := req.loop_id

  command_p.io.in.valid := state =/= idle && !io.wait_for_prev_loop && !skip
  command_p.io.in.bits.cmd := Mux(state === config, config_cmd, mvin_cmd)
  command_p.io.in.bits.dram_addr := dram_addr
  command_p.io.in.bits.spad_addr := spad_addr
  command_p.io.in.bits.I := I
  command_p.io.in.bits.J := J

  command_p.io.out.ready := io.cmd.ready && !io.rob_overloaded
  io.cmd.valid := command_p.io.out.valid && !io.rob_overloaded
  io.cmd.bits := command_p.io.out.bits.cmd
  when (command_p.io.out.bits.cmd.inst.funct === LOAD3_CMD) {
    val o = command_p.io.out.bits
    io.cmd.bits.rs1 := o.dram_addr
    val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType)
    mvin_cmd_rs2 := DontCare
    mvin_cmd_rs2.num_rows := o.I.asUInt()
    mvin_cmd_rs2.num_cols := o.J.asUInt()
    mvin_cmd_rs2.local_addr := cast_to_acc_addr(mvin_cmd_rs2.local_addr, o.spad_addr, accumulate = false.B, read_full = false.B)
    io.cmd.bits.rs2 := mvin_cmd_rs2.asUInt()
  }

  // Sending outputs
  when (skip) {
    state := idle
  }.elsewhen(command_p.io.in.fire) {
    when (state === config) {
      state := ld
    }.otherwise {

      val next_offset = floorAdd(offset, block_size.U, orows_ocols)
      val next_cb = floorAdd(cb, 1.U, ch_blk_num, next_offset === 0.U)

      offset := next_offset
      cb := next_cb
      
      state := Mux(next_cb === 0.U && next_offset === 0.U,
        idle, ld)
    }
  }

  // Accepting requests
  when (io.req.fire) {
    req := io.req.bits
    state := config

    cb := 0.U
    offset := 0.U

  }
}

class LoopConvDWLdInputReq(val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_acc_addr: Int, val concurrent_loops: Int)  extends Bundle {
  val outer_bounds = new LoopConvDWOuterBounds(iterator_bitwidth)
  val inner_bounds = new LoopConvDWInnerBounds(iterator_bitwidth)
  val derived_params = new LoopConvDWDerivedParams(iterator_bitwidth)
  val addr_start = UInt(log2Up(max_acc_addr).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
}

class LoopConvDWLdInput(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_addr: Int, input_w: Int,
                      max_block_len: Int, concurrent_loops: Int, latency: Int,
                      config_mvin_rs1_t: ConfigMvinRs1, mvin_rs2_t: MvinRs2)(implicit p: Parameters) extends Module {
  val MVIN_SCALE_IDENTITY = 0x3f800000.U // TODO get this from configs somehow

  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopConvDWLdInputReq(coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())
    val wait_for_prev_loop = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, config, ld = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopConvDWLdInputReq(coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops))
  import req.outer_bounds._
  import req.inner_bounds._
  import req.derived_params._

  dontTouch(req)

  // Derived parameters
  
  // Iterators
  val cb = Reg(SInt(iterator_bitwidth.W))
  val irow = Reg(SInt(iterator_bitwidth.W))
  val icol = Reg(SInt(iterator_bitwidth.W))

  dontTouch(cb)
  dontTouch(irow)
  dontTouch(icol)

  // Calculated params
  val irow_padded = irow +& upad.zext()
  val icol_padded = icol +& lpad.zext()
  val is_zeros = irow < 0.S || irow >= irows_unpadded.zext() || icol < 0.S || icol >= icols_unpadded.zext()

  val dram_stride = num_channels * (input_w/8).U
  // Addresses
  val dram_offset = ((cb * block_size.U) +& (irow * in_dim +& icol) * num_channels).asUInt()
  val dram_addr = Mux(is_zeros, 0.U, req.dram_addr + LoopConvDW.castDramOffset(dram_offset))
  val spad_addr = req.addr_start.zext() +& cb * ((irows_icols)) + irow_padded * (icols) +& icol_padded
  
  val I = MuxCase(
    Mux(icols_unpadded.zext() -& icol > block_size.S, block_size.S, icols_unpadded.zext() -& icol),
    Seq(
      (icol < 0.S) -> Mux((0.S-&icol) > block_size.S, block_size.S, 0.S-&icol),
      (icol >= icols_unpadded.zext()) -> Mux(icols_unpadded.zext() +& rpad.zext() -& icol > block_size.S, block_size.S, icols_unpadded.zext() +& rpad.zext() -& icol)
    )
  )
  val K = Mux(cb === (ch_blk_num - 1.U).asSInt(), chs.asSInt() - block_size.S * cb, block_size.S)

  class RoCCCommandWithAddr extends Bundle {
    val cmd = new RoCCCommand
    val dram_addr = UInt()
    val spad_addr = SInt()
    val I = SInt()
    val K = SInt()
  }
  val command_p = Module(new Pipeline[RoCCCommandWithAddr](new RoCCCommandWithAddr, latency)())
  // Commands
  val config_cmd = Wire(new RoCCCommand)
  config_cmd := DontCare
  config_cmd.inst.funct := CONFIG_CMD

  val config_cmd_rs1 = Wire(config_mvin_rs1_t.cloneType)
  config_cmd_rs1 := DontCare
  config_cmd_rs1.scale := MVIN_SCALE_IDENTITY
  config_cmd_rs1.stride := input_spad_stride
  config_cmd_rs1.pixel_repeats := 1.U
  config_cmd_rs1.state_id := 0.U
  config_cmd_rs1.shrink := 0.U
  config_cmd_rs1._unused := 1.U
  config_cmd.rs1 := config_cmd_rs1.asUInt()

  config_cmd.rs2 := dram_stride

  val mvin_cmd = Wire(new RoCCCommand)
  mvin_cmd := DontCare
  mvin_cmd.inst.funct := LOAD_CMD
  mvin_cmd.rs1 := 0.U // dram_addr
  mvin_cmd.rs2 := 0.U // mvin_cmd_rs2

  // Inputs and outputs
  io.req.ready := state === idle && !command_p.io.busy
  io.idle := state === idle && !command_p.io.busy
  io.loop_id := req.loop_id

  command_p.io.in.valid := state =/= idle && !io.wait_for_prev_loop
  command_p.io.in.bits.cmd := Mux(state === config, config_cmd, mvin_cmd)
  command_p.io.in.bits.dram_addr := dram_addr
  command_p.io.in.bits.spad_addr := spad_addr
  command_p.io.in.bits.I := I
  command_p.io.in.bits.K := K

  command_p.io.out.ready := io.cmd.ready && !io.rob_overloaded
  io.cmd.valid := command_p.io.out.valid && !io.rob_overloaded
  io.cmd.bits := command_p.io.out.bits.cmd
  when (command_p.io.out.bits.cmd.inst.funct === LOAD_CMD) {
    val o = command_p.io.out.bits
    io.cmd.bits.rs1 := o.dram_addr
    val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType)
    mvin_cmd_rs2 := DontCare
    mvin_cmd_rs2.num_rows := (o.I).asUInt()
    mvin_cmd_rs2.num_cols := o.K.asUInt()
    mvin_cmd_rs2.local_addr := cast_to_sp_addr(mvin_cmd_rs2.local_addr, o.spad_addr)
    io.cmd.bits.rs2 := mvin_cmd_rs2.asUInt()
  }

  // Sending outputs
  when(command_p.io.in.fire) {
    when (state === config) {
      state := ld
    }.otherwise {

      val next_icol = sFloorAdd(icol, I.asUInt(), (icols_unpadded +& rpad).zext(), 0.S-&lpad.zext())
      val next_irow = sFloorAdd(irow, 1.U, (irows_unpadded +& dpad).zext(), 0.S -&upad.zext(),
        next_icol === 0.S-&lpad.zext())

        val next_cb = sFloorAdd(cb, 1.U, ch_blk_num.zext(), 0.S,
        next_irow === 0.S-&upad.zext() && next_icol === 0.S-&lpad.zext())
      icol := next_icol
      irow := next_irow
      cb := next_cb

      state := Mux(next_irow === 0.S-&upad.zext() && next_icol === 0.S-&lpad.zext() && next_cb === 0.S,
        idle, ld)
    }
  }

  // Accepting requests
  when (io.req.fire) {
    req := io.req.bits
    state := config
    irow := 0.S -& (io.req.bits.inner_bounds.upad).zext()
    icol := 0.S -& (io.req.bits.inner_bounds.lpad).zext()
    cb := 0.S
  }
}

class LoopConvDWLdWeightReq(val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_addr: Int, val concurrent_loops: Int)  extends Bundle {
  val outer_bounds = new LoopConvDWOuterBounds(iterator_bitwidth)
  val inner_bounds = new LoopConvDWInnerBounds(iterator_bitwidth)
  val derived_params = new LoopConvDWDerivedParams(iterator_bitwidth)
  val addr_end = UInt(log2Up(max_addr+1).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
}

class LoopConvDWLdWeight(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_addr: Int, input_w: Int,
                       max_block_len: Int, concurrent_loops: Int, latency: Int,
                       config_mvin_rs1_t: ConfigMvinRs1, mvin_rs2_t: MvinRs2)(implicit p: Parameters) extends Module {
  val MVIN_SCALE_IDENTITY = 0x3f800000.U // TODO get this from configs somehow

  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopConvDWLdWeightReq(coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())
    val wait_for_prev_loop = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, config, ld = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopConvDWLdWeightReq(coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops))
  import req.outer_bounds._
  import req.inner_bounds._
  import req.derived_params._

  dontTouch(req)

  // Derived parameters
  val B_rows = chs
  val addr_start = req.addr_end - B_rows

  val dram_stride = block_size.U * (input_w/8).U

  // Iterators
  val ch = Reg(UInt(iterator_bitwidth.W))

  dontTouch(ch)

  val dram_offset = (ch * block_size.U) * (input_w/8).U
  val dram_addr = req.dram_addr + LoopConvDW.castDramOffset(dram_offset)

  val spad_addr = addr_start + ch
  // Sizes
  val J = block_size.U
  val K = Mux(chs - ch > block_size.U, block_size.U, chs - ch)

  class RoCCCommandWithAddr extends Bundle {
    val cmd = new RoCCCommand
    val dram_addr = UInt()
    val spad_addr = UInt()
    val K = UInt()
    val J = UInt()
  }
  val command_p = Module(new Pipeline[RoCCCommandWithAddr](new RoCCCommandWithAddr, latency)())

  // Commands
  val config_cmd = Wire(new RoCCCommand)
  config_cmd := DontCare
  config_cmd.inst.funct := CONFIG_CMD

  val config_cmd_rs1 = Wire(config_mvin_rs1_t.cloneType)
  config_cmd_rs1 := DontCare
  config_cmd_rs1.scale := MVIN_SCALE_IDENTITY
  config_cmd_rs1.stride := req.derived_params.weight_spad_stride
  config_cmd_rs1.pixel_repeats := 1.U
  config_cmd_rs1.state_id := 1.U
  config_cmd_rs1.shrink := 0.U
  config_cmd_rs1._unused := 1.U
  config_cmd.rs1 := config_cmd_rs1.asUInt

  config_cmd.rs2 := dram_stride

  val mvin_cmd = Wire(new RoCCCommand)
  mvin_cmd := DontCare
  mvin_cmd.inst.funct := LOAD2_CMD
  mvin_cmd.rs1 := 0.U // dram_addr
  mvin_cmd.rs2 := 0.U // mvin_cmd_rs2

  // Inputs and outputs
  io.req.ready := state === idle && !command_p.io.busy
  io.idle := state === idle && !command_p.io.busy
  io.loop_id := req.loop_id

  command_p.io.in.valid := state =/= idle && !io.wait_for_prev_loop
  command_p.io.in.bits.cmd := Mux(state === config, config_cmd, mvin_cmd)
  command_p.io.in.bits.dram_addr := dram_addr
  command_p.io.in.bits.spad_addr := spad_addr
  command_p.io.in.bits.K := K
  command_p.io.in.bits.J := J

  command_p.io.out.ready := io.cmd.ready && !io.rob_overloaded
  io.cmd.valid := command_p.io.out.valid && !io.rob_overloaded
  io.cmd.bits := command_p.io.out.bits.cmd
  when (command_p.io.out.bits.cmd.inst.funct === LOAD2_CMD) {
    val o = command_p.io.out.bits
    io.cmd.bits.rs1 := o.dram_addr
    val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType)
    mvin_cmd_rs2 := DontCare
    mvin_cmd_rs2.num_rows := o.K
    mvin_cmd_rs2.num_cols := o.J
    mvin_cmd_rs2.local_addr := cast_to_sp_addr(mvin_cmd_rs2.local_addr, o.spad_addr)
    io.cmd.bits.rs2 := mvin_cmd_rs2.asUInt()
  }

  // Sending outputs
  when(command_p.io.in.fire) {
    when (state === config) {
      state := ld
    }.otherwise {
      val next_ch = floorAdd(ch, block_size.U, chs)

      // val next_ch = 0.U

      ch := next_ch

      state := Mux(next_ch === 0.U,
        idle, ld)

    }
  }

  // Accepting requests
  when (io.req.fire) {
    req := io.req.bits
    state := config

    ch := 0.U
  }
}

class LoopConvDWExecuteReq(val iterator_bitwidth: Int, val max_addr: Int, val max_acc_addr: Int, val concurrent_loops: Int)  extends Bundle {
  val outer_bounds = new LoopConvDWOuterBounds(iterator_bitwidth)
  val inner_bounds = new LoopConvDWInnerBounds(iterator_bitwidth)
  val derived_params = new LoopConvDWDerivedParams(iterator_bitwidth)
  val a_addr_start = UInt(log2Up(max_addr).W)
  val b_addr_end = UInt(log2Up(max_addr+1).W)
  val c_addr_start = UInt(log2Up(max_acc_addr).W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
}

class LoopConvDWExecute(block_size: Int, iterator_bitwidth: Int, max_addr: Int,
                      max_acc_addr: Int, concurrent_loops: Int, latency: Int,
                      config_ex_rs1_t: ConfigExRs1, preload_rs1_t: PreloadRs, preload_rs2_t: PreloadRs,
                      compute_rs1_t: ComputeRs, compute_rs2_t: ComputeRs)(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopConvDWExecuteReq(iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val lda_completed = Input(Bool())
    val ldb_completed = Input(Bool())
    val ldd_completed = Input(Bool())

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, config, config2, pre, comp = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopConvDWExecuteReq(iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops))

  import req.outer_bounds._
  import req.inner_bounds._
  import req.derived_params._

  dontTouch(req)

  // Derived parameters
  val B_rows = chs

  val a_addr_start = req.a_addr_start
  val b_addr_start = req.b_addr_end - B_rows
  val c_addr_start = /*(BigInt(3) << 30).U |*/ req.c_addr_start

  // Iterators
  val cb = Reg(UInt(iterator_bitwidth.W))
  val i = Reg(UInt(iterator_bitwidth.W))

  dontTouch(cb)
  dontTouch(i)


  val I = atomic_irows_icols * 2.U
  val J = Mux(cb === ch_blk_num -1.U, chs - block_size.U * cb, block_size.U)
  val K = Mux(cb === ch_blk_num -1.U, chs - block_size.U * cb, block_size.U)
  val L = frame_size



  class RoCCCommandWithAddr extends Bundle {
    val cmd = new RoCCCommand
    val a_addr = UInt()
    val b_addr = UInt()
    val c_addr = UInt()
    val I = UInt()
    val J = UInt()
    val K = UInt()
    val L = UInt()
    val new_weights = Bool()
  }
  val command_p = Module(new Pipeline[RoCCCommandWithAddr](new RoCCCommandWithAddr, latency)())

  //
  //
  // Commands
  val config_cmd = Wire(new RoCCCommand)
  config_cmd := DontCare
  config_cmd.inst.funct := CONFIG_CMD

  val config_cmd_rs1 = Wire(config_ex_rs1_t.cloneType)
  config_cmd_rs1 := DontCare
  config_cmd_rs1.acc_scale := 0.U
  config_cmd_rs1.a_stride := stride
  config_cmd_rs1.b_transpose := 0.U
  config_cmd_rs1.a_transpose := 0.U
  config_cmd_rs1.set_only_strides := 0.U
  config_cmd_rs1.activation := 0.U
  config_cmd_rs1.set_only_strides := 1.U
  config_cmd_rs1.dataflow := 1.U //WEIGHT_STATIONARY
  config_cmd_rs1.cmd_type := CONFIG_EX

  val config_cmd_rs2 = Wire(new ConfigExRs2)
  config_cmd_rs2 := DontCare
  config_cmd_rs2.c_stride := 1.U
  // config_cmd_rs2.relu6_shift := 0.U
  config_cmd_rs2.in_shift := 0.U

  config_cmd.rs1 := config_cmd_rs1.asUInt()
  config_cmd.rs2 := config_cmd_rs2.asUInt()

  val config_cmd_im2col = Wire(new RoCCCommand)
  config_cmd_im2col := DontCare
  config_cmd_im2col.inst.funct := CONFIG_CMD

  val config_cmd_dwconv_rs1 = Wire(new ConfigDWConvRs1)
  config_cmd_dwconv_rs1 := DontCare
  config_cmd_dwconv_rs1.orows := orows
  config_cmd_dwconv_rs1.ocols := ocols
  config_cmd_dwconv_rs1.icols := icols
  config_cmd_dwconv_rs1.irows := irows
  config_cmd_dwconv_rs1.atomic_orows := Mux(i + atomic_orows >= orows, orows - i, atomic_orows)
  config_cmd_dwconv_rs1.ch_blk_num := ch_blk_num
  config_cmd_dwconv_rs1.is_dwconv_depthwise := 1.U
  config_cmd_dwconv_rs1.cmd_type := CONFIG_IM2COL

  val config_cmd_dwconv_rs2 = Wire(new ConfigDWConvRs2)
  config_cmd_dwconv_rs2 := DontCare
  config_cmd_dwconv_rs2.frame_height := frame_height 
  config_cmd_dwconv_rs2.frame_size := frame_size 
  config_cmd_dwconv_rs2.dw_atomic_orows_phase_total :=  Mux(i + atomic_orows*2.U >= orows, orows - i, atomic_orows * 2.U)  //this is added to skip the exceeded outputs rows

  config_cmd_im2col.rs1 := config_cmd_dwconv_rs1.asUInt()
  config_cmd_im2col.rs2 := config_cmd_dwconv_rs2.asUInt()

  val compute_ch_num = Mux (cb === ch_blk_num - 1.U, chs - block_size.U * cb, block_size.U)
  dontTouch(compute_ch_num)
  val new_weights = RegInit(true.B)
  val a_addr = a_addr_start +& (cb * (irows_icols)) + ((icols) * stride * i)
  val b_addr = b_addr_start +& (cb * block_size.U)
  val c_addr = c_addr_start +& (cb * orows_ocols) + i
  //
  //

  val pre_cmd = Wire(new RoCCCommand) // preload
  pre_cmd := DontCare
  pre_cmd.inst.funct := PRELOAD_CMD

  val comp_cmd = Wire(new RoCCCommand()) // compute.preloaded
  comp_cmd := DontCare
  comp_cmd.inst.funct := Mux(new_weights, COMPUTE_AND_FLIP_CMD, COMPUTE_AND_STAY_CMD)
  
  val ld_ahead = io.lda_completed && io.ldb_completed && io.ldd_completed

  // Inputs and outputs
  io.req.ready := state === idle && !command_p.io.busy
  io.idle := state === idle && !command_p.io.busy
  io.loop_id := req.loop_id

  command_p.io.in.valid := state =/= idle && ld_ahead // && !skip_iteration
  command_p.io.in.bits.cmd := MuxCase(config_cmd, Seq((state === config2) -> config_cmd_im2col, (state === pre) -> pre_cmd, (state === comp) -> comp_cmd))
  command_p.io.in.bits.a_addr := a_addr
  command_p.io.in.bits.b_addr := b_addr
  command_p.io.in.bits.c_addr := c_addr
  command_p.io.in.bits.I := I
  command_p.io.in.bits.J := J
  command_p.io.in.bits.K := K
  command_p.io.in.bits.L := L
  command_p.io.in.bits.new_weights := new_weights

  command_p.io.out.ready := io.cmd.ready && !io.rob_overloaded
  io.cmd.valid := command_p.io.out.valid && !io.rob_overloaded
  io.cmd.bits := command_p.io.out.bits.cmd
  when (command_p.io.out.bits.cmd.inst.funct === PRELOAD_CMD) {
    val o = command_p.io.out.bits

    val pre_cmd_rs1 = Wire(preload_rs1_t.cloneType)
    pre_cmd_rs1 := DontCare
    pre_cmd_rs1.num_rows := o.K.asUInt()
    pre_cmd_rs1.num_cols := o.L.asUInt()
    pre_cmd_rs1.local_addr := Mux(o.new_weights, cast_to_sp_addr(pre_cmd_rs1.local_addr, o.b_addr),
      garbage_addr(pre_cmd_rs1.local_addr))

    val pre_cmd_rs2 = Wire(preload_rs2_t.cloneType)
    pre_cmd_rs2 := DontCare
    pre_cmd_rs2.num_rows := o.I.asUInt()
    pre_cmd_rs2.num_cols := o.J.asUInt()
    pre_cmd_rs2.local_addr := cast_to_acc_addr(pre_cmd_rs2.local_addr, o.c_addr, accumulate = true.B, read_full = false.B)

    io.cmd.bits.rs1 := pre_cmd_rs1.asUInt()
    io.cmd.bits.rs2 := pre_cmd_rs2.asUInt()
  }.elsewhen(command_p.io.out.bits.cmd.inst.funct =/= CONFIG_CMD) {
    val o = command_p.io.out.bits
    val comp_cmd_rs1 = Wire(compute_rs1_t.cloneType)
    comp_cmd_rs1 := DontCare
    comp_cmd_rs1.num_rows := o.I.asUInt()
    comp_cmd_rs1.num_cols := o.K.asUInt()
    comp_cmd_rs1.local_addr := cast_to_sp_addr(comp_cmd_rs1.local_addr, o.a_addr)

    val comp_cmd_rs2 = Wire(compute_rs2_t.cloneType)
    comp_cmd_rs2 := DontCare
    comp_cmd_rs2.num_rows := o.I.asUInt()
    comp_cmd_rs2.num_cols := o.J.asUInt()
    comp_cmd_rs2.local_addr := garbage_addr(comp_cmd_rs2.local_addr)

    io.cmd.bits.rs1 := comp_cmd_rs1.asUInt()
    io.cmd.bits.rs2 := comp_cmd_rs2.asUInt()
  }

  // Updating "new_weights"
  // when (state === comp && command_p.io.in.fire) {
  //   new_weights := false.B
  // }

  // Sending outputs
  when (command_p.io.in.fire) { // || skip_iteration
    when (state === config) {
      state := config2
    }.elsewhen (state === config2) {
      state := pre
    }.elsewhen (state === pre){
      state := comp
    }.otherwise {
      val next_i = floorAdd(i, atomic_orows*2.U, orows) // *2 for dual phase processing
      val next_cb = floorAdd(cb, 1.U, ch_blk_num, next_i === 0.U)

      i := next_i
      cb := next_cb

      state := Mux(next_cb === 0.U && next_i === 0.U, idle, config)

      new_weights := Mux(next_i === 0.U, true.B, false.B)
    }
  }

  // Accepting requests
  when (io.req.fire) {
    req := io.req.bits
    state := config

    cb := 0.U
    i := 0.U

    new_weights := true.B
  }
}

class LoopConvDWStReq(val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_acc_addr: Int, val concurrent_loops: Int)  extends Bundle {
  val outer_bounds = new LoopConvDWOuterBounds(iterator_bitwidth)
  val inner_bounds = new LoopConvDWInnerBounds(iterator_bitwidth)
  val derived_params = new LoopConvDWDerivedParams(iterator_bitwidth)
  val addr_start = UInt(log2Up(max_acc_addr).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val activation = UInt(2.W) // TODO magic number
  val loop_id = UInt(log2Up(concurrent_loops).W)
}

class LoopConvDWSt(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_acc_addr: Int, input_w: Int, concurrent_loops: Int, latency: Int, config_mvout_rs2_t: ConfigMvoutRs2, mvout_rs2_t: MvoutRs2)(implicit p: Parameters) extends Module {
  val ACC_SCALE_NO_CHANGE = ~(0.U(32.W)) // TODO get this from ISA description somehow

  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopConvDWStReq(coreMaxAddrBits, iterator_bitwidth: Int, max_acc_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val ex_completed = Input(Bool())

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, st = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopConvDWStReq(coreMaxAddrBits, iterator_bitwidth: Int, max_acc_addr, concurrent_loops))
  import req.outer_bounds._
  import req.inner_bounds._
  import req.derived_params._
  dontTouch(req)
  val acc_addr_start = req.addr_start

  // Derived parameters
  val skip = req.dram_addr === 0.U

  // Iterators
  val orow = Reg(UInt(iterator_bitwidth.W))
  val ocol = Reg(UInt(iterator_bitwidth.W))
  val ch = Reg(UInt(iterator_bitwidth.W))
  val cb = Reg(UInt(iterator_bitwidth.W))

  dontTouch(cb)
  dontTouch(orow)
  dontTouch(ocol)
  // Addresses
  val dram_offset = ((cb * block_size.U) + (orow * out_dim + ocol) * num_channels) * (input_w/8).U
  val dram_addr = req.dram_addr + LoopConvDW.castDramOffset(dram_offset)
  val spad_addr = acc_addr_start +& (cb * orows_ocols) +& ocol * orows +& orow

  // Sizes
  // val I = Mux(ocols - ocol > block_size.U, block_size.U, ocols - ocol)
  val I = Mux(orows - orow > block_size.U, block_size.U, orows - orow)
  val J = Mux(cb === ch_blk_num - 1.U, chs - block_size.U * cb, block_size.U)

  val channels = J

  class RoCCCommandWithAddr extends Bundle {
    val cmd = new RoCCCommand
    val dram_addr = UInt()
    val spad_addr = UInt()
    val channels = UInt()
    val I = UInt()
    val J = UInt()
  }
  val command_p = Module(new Pipeline[RoCCCommandWithAddr](new RoCCCommandWithAddr, latency)())
  // Commands
  val mvout_cmd = Wire(new RoCCCommand)
  mvout_cmd := DontCare
  mvout_cmd.inst.funct := STORE_CMD
  mvout_cmd.rs1 := 0.U // dram_addr
  mvout_cmd.rs2 := 0.U // mvout_cmd_rs2



  // Inputs and outputs
  io.req.ready := state === idle && !command_p.io.busy
  io.idle := state === idle && !command_p.io.busy
  io.loop_id := req.loop_id

  command_p.io.in.valid := state =/= idle && !skip && io.ex_completed
  command_p.io.in.bits.cmd := mvout_cmd
  command_p.io.in.bits.dram_addr := dram_addr
  command_p.io.in.bits.spad_addr := spad_addr
  command_p.io.in.bits.channels := channels
  command_p.io.in.bits.I := I
  command_p.io.in.bits.J := J

  command_p.io.out.ready := io.cmd.ready && !io.rob_overloaded
  io.cmd.valid := command_p.io.out.valid && !io.rob_overloaded //false.B
  io.cmd.bits := command_p.io.out.bits.cmd
  when (command_p.io.out.bits.cmd.inst.funct === STORE_CMD) {
    val o = command_p.io.out.bits
    val mvout_cmd_rs2 = Wire(mvout_rs2_t.cloneType)
    mvout_cmd_rs2 := DontCare
    mvout_cmd_rs2.num_rows := o.I.asUInt()
    mvout_cmd_rs2.num_cols := o.J.asUInt()
    mvout_cmd_rs2.local_addr := cast_to_acc_addr(mvout_cmd_rs2.local_addr, o.spad_addr, accumulate = false.B, read_full = false.B)
    io.cmd.bits.rs1 := o.dram_addr
    io.cmd.bits.rs2 := mvout_cmd_rs2.asUInt()
  }

  // Sending outputs
  when (skip) {
    state := idle
  }.elsewhen(command_p.io.in.fire) {
      val next_ch = floorAdd(ch, block_size.U, chs)
      val next_orow = floorAdd(orow, block_size.U, orows)
      val next_ocol = floorAdd(ocol, 1.U, ocols, next_orow === 0.U)
      val next_cb = floorAdd(cb, 1.U, ch_blk_num, next_orow === 0.U && next_ocol === 0.U)

      ocol := next_ocol
      orow := next_orow
      cb := next_cb

      state := Mux(next_orow === 0.U && next_ocol === 0.U && next_cb === 0.U,
        idle, st)
  }
  //idle, st
  // Accepting requests
  when (io.req.fire) {
    req := io.req.bits
    state := st

    cb := 0.U
    orow := 0.U
    ocol := 0.U
    ch := 0.U
  }
}

class LoopConvDWState(val block_size: Int, val iterator_bitwidth: Int, val coreMaxAddrBits: Int, val max_addr: Int, val max_acc_addr: Int) extends Bundle {
  val outer_bounds = new LoopConvDWOuterBounds(iterator_bitwidth)
  val inner_bounds = new LoopConvDWInnerBounds(iterator_bitwidth)

  val bias_dram_addr = UInt(coreMaxAddrBits.W)
  val weights_dram_addr = UInt(coreMaxAddrBits.W)
  val input_dram_addr = UInt(coreMaxAddrBits.W)
  val output_dram_addr = UInt(coreMaxAddrBits.W)

  val no_bias = Bool()
  val activation = UInt(2.W) // TODO magic number
  
  val configured = Bool()

  val running = Bool()

  val ld_bias_started = Bool()
  val ld_input_started = Bool()
  val ld_weights_started = Bool()
  val ex_started = Bool()
  val st_started = Bool()

  val ld_bias_completed = Bool()
  val ld_input_completed = Bool()
  val ld_weights_completed = Bool()
  val ex_completed = Bool()
  val st_completed = Bool()

  def all_completed(dummy: Int=0): Bool = ld_bias_completed && ld_input_completed && ld_weights_completed && ex_completed && st_completed

  val a_addr_start = UInt(log2Up(max_addr).W)
  val b_addr_end = UInt(log2Up(max_addr+1).W)

  def derived_params(dummy: Int=0): LoopConvDWDerivedParams = {
    import outer_bounds.{stride}
    import inner_bounds.{orows, ocols, irows, icols, krows, kcols, upad, dpad, lpad, rpad, chs}

    val result = Wire(new LoopConvDWDerivedParams(iterator_bitwidth))

    result.irows_unpadded := irows -& upad -& dpad
    result.icols_unpadded := icols -& lpad -& rpad

    result.bias_spad_stride := block_size.U //orows * ocols
    result.input_spad_stride := block_size.U
    result.weight_spad_stride := block_size.U 

    result
  }

  def reset(): Unit = {
    configured := false.B

    running := false.B

    ld_bias_started := false.B
    ld_input_started := false.B
    ld_weights_started := false.B
    ex_started := false.B
    st_started := false.B

    ld_bias_completed := false.B
    ld_input_completed := false.B
    ld_weights_completed := false.B
    ex_completed := false.B
    st_completed := false.B
  }
}

class LoopConvDW (block_size: Int, coreMaxAddrBits: Int, reservation_station_size: Int, max_lds: Int, max_exs: Int, max_sts: Int,
                max_addr: Int, max_acc_addr: Int, input_w: Int, acc_w: Int, dma_max_bytes: Int,
                config_mvin_rs1_t: ConfigMvinRs1, mvin_rs2_t: MvinRs2, config_mvout_rs2_t: ConfigMvoutRs2, mvout_rs2_t: MvoutRs2,
                config_ex_rs1_t: ConfigExRs1, preload_rs1_t: PreloadRs, preload_rs2_t: PreloadRs,
                compute_rs1_t: ComputeRs, compute_rs2_t: ComputeRs)
  (implicit p: Parameters) extends Module {
  val iterator_bitwidth = 16

  val max_block_len = (dma_max_bytes / (block_size * (input_w / 8))) max 1
  val max_block_len_acc = (dma_max_bytes / (block_size * (acc_w / 8))) max 1

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new GemminiCmd(reservation_station_size)))
    val out = Decoupled(new GemminiCmd(reservation_station_size))
    val ld_completed = Input(UInt(log2Up(reservation_station_size+1).W))
    val st_completed = Input(UInt(log2Up(reservation_station_size+1).W))
    val ex_completed = Input(UInt(log2Up(reservation_station_size+1).W))
    val busy = Output(Bool())
  })

  // Create states
  val concurrent_loops = 2
  val loops = Reg(Vec(concurrent_loops, new LoopConvDWState(block_size, iterator_bitwidth, coreMaxAddrBits, max_addr, max_acc_addr)))
  val head_loop_id = RegInit(0.U(log2Up(concurrent_loops).W))
  val tail_loop_id = (~head_loop_id).asUInt() // This is the loop that we always try to configure if available
  val head_loop = loops(head_loop_id)
  val tail_loop = loops(tail_loop_id)

  val loop_configured = loops.map(_.configured).reduce(_ || _)

  val loop_being_configured_id = Mux(head_loop.configured, tail_loop_id, head_loop_id)
  val loop_being_configured = loops(loop_being_configured_id)

  // Create inner modules
  val latency = 4
  val ld_bias = Module(new LoopConvDWLdBias(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, acc_w, max_block_len_acc, concurrent_loops, latency, config_mvin_rs1_t, mvin_rs2_t))
  val ld_input = Module(new LoopConvDWLdInput(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, input_w, max_block_len, concurrent_loops, latency, config_mvin_rs1_t, mvin_rs2_t))
  val ld_weights = Module(new LoopConvDWLdWeight(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, input_w, max_block_len, concurrent_loops, latency, config_mvin_rs1_t, mvin_rs2_t))
  val ex = Module(new LoopConvDWExecute(block_size, iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops, latency, config_ex_rs1_t, preload_rs1_t, preload_rs2_t, compute_rs1_t, compute_rs2_t))
  val st = Module(new LoopConvDWSt(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, input_w, concurrent_loops, latency, config_mvout_rs2_t, mvout_rs2_t))

  // Create command queue
  val cmd = Queue(io.in)

  io.busy := cmd.valid || loop_configured

  // Create arbiter
  val arb = Module(new Arbiter(new RoCCCommand, 5))
  arb.io.in(0) <> st.io.cmd
  arb.io.in(1) <> ex.io.cmd
  arb.io.in(2) <> ld_bias.io.cmd
  arb.io.in(3) <> ld_weights.io.cmd
  arb.io.in(4) <> ld_input.io.cmd
  val unrolled_cmd = arb.io.out

  // Create reservation station utilization counters
  val ld_utilization = RegInit(0.U(log2Up(max_lds+1).W))
  val st_utilization = RegInit(0.U(log2Up(max_sts+1).W))
  val ex_utilization = RegInit(0.U(log2Up(max_exs+1).W))

  ld_utilization := ld_utilization +& (ld_bias.io.cmd.fire || ld_weights.io.cmd.fire || ld_input.io.cmd.fire) -& io.ld_completed
  st_utilization := st_utilization +& st.io.cmd.fire -& io.st_completed
  ex_utilization := ex_utilization +& ex.io.cmd.fire -& io.ex_completed

  assert(ld_utilization >= io.ld_completed, "ld utilization underflow")
  assert(st_utilization >= io.st_completed, "st utilization underflow")
  assert(ex_utilization >= io.ex_completed, "ex utilization underflow")

  // Wire up unrolled command output
  val is_loop_run_cmd = cmd.bits.cmd.inst.funct === LOOP_DW_CONV_WS
  val is_loop_config_cmd = cmd.bits.cmd.inst.funct >= LOOP_DW_CONV_WS_CONFIG_1 && cmd.bits.cmd.inst.funct <= LOOP_DW_CONV_WS_CONFIG_6
  val is_loop_cmd = is_loop_run_cmd || is_loop_config_cmd

  io.out.bits.cmd := Mux(loop_configured, unrolled_cmd.bits, cmd.bits.cmd)
  io.out.bits.cmd.status := cmd.bits.cmd.status // TODO This is not guaranteed to be the correct fix! We must fix this
  io.out.bits.rob_id := DontCare
  io.out.bits.from_matmul_fsm := Mux(loop_configured, false.B, cmd.bits.from_matmul_fsm)
  io.out.bits.from_conv_fsm := Mux(loop_configured, false.B, cmd.bits.from_conv_fsm)
  io.out.bits.from_conv_dw_fsm := Mux(loop_configured, true.B, cmd.bits.from_conv_dw_fsm)

  io.out.valid := Mux(loop_configured, unrolled_cmd.valid, cmd.valid && !is_loop_config_cmd && !is_loop_run_cmd)

  cmd.ready := Mux(is_loop_cmd, !loop_being_configured.configured, !loop_configured && io.out.ready)
  arb.io.out.ready := io.out.ready

  when(io.out.fire) {
    printf("io.out.fire %x %x %x\n",io.out.bits.cmd.inst.funct,io.out.bits.cmd.rs1,io.out.bits.cmd.rs2)
  }

  // Wire up waiting-for-loads signals
  val ex_is_waiting_for_loads = loops(ex.io.loop_id).ex_started && !loops(ex.io.loop_id).ex_completed &&
    !(loops(ex.io.loop_id).ld_input_completed && loops(ex.io.loop_id).ld_weights_completed &&
      loops(ex.io.loop_id).ld_bias_completed)

  ld_bias.io.wait_for_prev_loop := ex_is_waiting_for_loads && ld_bias.io.loop_id =/= ex.io.loop_id
  ld_weights.io.wait_for_prev_loop := ex_is_waiting_for_loads && ld_weights.io.loop_id =/= ex.io.loop_id
  ld_input.io.wait_for_prev_loop := ex_is_waiting_for_loads && ld_input.io.loop_id =/= ex.io.loop_id

  // Wire up overloaded signals
  ld_bias.io.rob_overloaded := ld_utilization >= max_lds.U
  ld_input.io.rob_overloaded := ld_utilization >= max_lds.U
  ld_weights.io.rob_overloaded := ld_utilization >= max_lds.U
  ex.io.rob_overloaded := ex_utilization >= max_exs.U
  st.io.rob_overloaded := st_utilization >= max_sts.U

  // Wire up iterator inputs
  ex.io.lda_completed := (ld_input.io.loop_id =/= ex.io.loop_id) || ld_input.io.idle
  ex.io.ldb_completed := (ld_weights.io.loop_id =/= ex.io.loop_id) || ld_weights.io.idle
  ex.io.ldd_completed := (ld_bias.io.loop_id =/= ex.io.loop_id) || ld_bias.io.idle
  st.io.ex_completed := (ex.io.loop_id =/= st.io.loop_id) || ex.io.idle

  // Create config registers
  when(cmd.valid && is_loop_cmd && !loop_being_configured.configured) {

//                              //     10,       12,     10,      2,           3,        3,    3,     12    1,    1,    1,    1,    9,       9,       18   ,    9    ,  9  ,    18,            9,            8,         17,                8,           6,                          1,        2,
// #define gemmini_loop_dwconv_dw_ws(in_dim, num_channels, out_dim, stride,  kernel_dim, krows, kcols, chs, lpad, rpad, upad, dpad, orows, ocols, orows_ocols, irows, icols, irows_icols,  ch_blk_num, atomic_orows, atomic_irows_icols, frame_size, frame_height, weights, output, bias, input, no_bias, activation) \
//   { \
//     ROCC_INSTRUCTION_RS1_RS2(XCUSTOM_ACC, ( \
//                                             ((uint64_t)(in_dim) << 54) | ((uint64_t)(num_channels) << 42) | ((uint64_t)(out_dim) << 32) | ((uint64_t)(stride) << 30) | \
//                                             ((uint64_t)(kernel_dim) << 27) | ((uint64_t)(krows) << 24) | ((uint64_t)(kcols) << 21) | ((uint64_t)(chs) << 9) | \
//                                             ((uint64_t)(lpad) << 8) | ((uint64_t)(rpad) << 7)| ((uint64_t)(upad) << 6) | ((uint64_t)(dpad) << 5) \
//                                           ), \
//                                           ( \
//                                             ((uint64_t)(orows) << 55) | ((uint64_t)(ocols) << 46) | ((uint64_t)(orows_ocols) << 28) | \
//                                             ((uint64_t)(irows) << 19) | ((uint64_t)(icols) << 10) \
//                                           ), \
//                                            k_LOOP_DW_CONV_WS_CONFIG_1) \
//     ROCC_INSTRUCTION_RS1_RS2(XCUSTOM_ACC, ( \
//                                             ((uint64_t)(irows_icols) << 46) | ((uint64_t)(ch_blk_num) << 37) | ((uint64_t)(atomic_orows) << 29) | ((uint64_t)(atomic_irows_icols) << 12) | \
//                                             ((uint64_t)(frame_size) << 4) | ((uint64_t)(no_bias) << 3) | ((uint64_t)(activation) << 1)  \
//                                           ), \
//                                           ( \
//                                             ((uint64_t)(frame_height) << 58)  \
//                                           ), \
//                                            k_LOOP_DW_CONV_WS_CONFIG_2) \
//     ROCC_INSTRUCTION_RS1_RS2(XCUSTOM_ACC, weights, \
//       output, k_LOOP_DW_CONV_WS_CONFIG_3) \
//     ROCC_INSTRUCTION_RS1_RS2(XCUSTOM_ACC, bias, \
//       input, k_LOOP_DW_CONV_WS) \
//   }

    switch (cmd.bits.cmd.inst.funct) {
      is (LOOP_DW_CONV_WS_CONFIG_1) {
        loop_being_configured.outer_bounds.in_dim := cmd.bits.cmd.rs1(63, 54)
        loop_being_configured.outer_bounds.num_channels := cmd.bits.cmd.rs1(53, 42)
        loop_being_configured.outer_bounds.out_dim := cmd.bits.cmd.rs1(41, 32)
        loop_being_configured.outer_bounds.stride := cmd.bits.cmd.rs1(31,30)
        loop_being_configured.outer_bounds.kernel_dim := cmd.bits.cmd.rs1(29, 27)
        
        loop_being_configured.inner_bounds.krows := cmd.bits.cmd.rs1(26, 24)
        loop_being_configured.inner_bounds.kcols := cmd.bits.cmd.rs1(23, 21)
        loop_being_configured.inner_bounds.chs := cmd.bits.cmd.rs1(20, 9)
        loop_being_configured.inner_bounds.lpad := cmd.bits.cmd.rs1(8)
        loop_being_configured.inner_bounds.rpad := cmd.bits.cmd.rs1(7)
        loop_being_configured.inner_bounds.upad := cmd.bits.cmd.rs1(6)
        loop_being_configured.inner_bounds.dpad := cmd.bits.cmd.rs1(5)

        loop_being_configured.inner_bounds.orows := cmd.bits.cmd.rs2(63,55)
        loop_being_configured.inner_bounds.ocols := cmd.bits.cmd.rs2(54,46)
        loop_being_configured.inner_bounds.orows_ocols := cmd.bits.cmd.rs2(45,28)
        loop_being_configured.inner_bounds.irows := cmd.bits.cmd.rs2(27,19)
        loop_being_configured.inner_bounds.icols := cmd.bits.cmd.rs2(18,10)
      }
      is (LOOP_DW_CONV_WS_CONFIG_2) {
        loop_being_configured.inner_bounds.irows_icols := cmd.bits.cmd.rs1(63,46)
        loop_being_configured.outer_bounds.ch_blk_num := cmd.bits.cmd.rs1(45,37)
        loop_being_configured.inner_bounds.atomic_orows := cmd.bits.cmd.rs1(36,29)
        loop_being_configured.inner_bounds.atomic_irows_icols := cmd.bits.cmd.rs1(28,12)
        loop_being_configured.outer_bounds.frame_size := cmd.bits.cmd.rs1(11,4)
        loop_being_configured.no_bias := cmd.bits.cmd.rs1(3)
        loop_being_configured.activation := cmd.bits.cmd.rs1(2,1)
        loop_being_configured.outer_bounds.frame_height := cmd.bits.cmd.rs2(63,58)
      }
      is (LOOP_DW_CONV_WS_CONFIG_3) {
        loop_being_configured.weights_dram_addr := cmd.bits.cmd.rs1
        loop_being_configured.output_dram_addr := cmd.bits.cmd.rs2
      }

      is (LOOP_DW_CONV_WS) {
        loop_being_configured.bias_dram_addr := cmd.bits.cmd.rs1
        loop_being_configured.input_dram_addr := cmd.bits.cmd.rs2

        loop_being_configured.configured := true.B
      }
    }
  }

  // Wire up request signals
  val ld_bias_addr_start = RegInit(0.U(log2Up(max_acc_addr).W))
  val ex_c_addr_start = RegInit(0.U(log2Up(max_acc_addr).W))
  val st_addr_start = RegInit(0.U(log2Up(max_acc_addr).W))

  val loop_requesting_ld_bias_id = Mux(head_loop.ld_bias_started, tail_loop_id, head_loop_id)
  val loop_requesting_ld_bias = loops(loop_requesting_ld_bias_id)
  ld_bias.io.req.bits.outer_bounds := loop_requesting_ld_bias.outer_bounds
  ld_bias.io.req.bits.inner_bounds := loop_requesting_ld_bias.inner_bounds
  ld_bias.io.req.bits.derived_params := loop_requesting_ld_bias.derived_params()
  ld_bias.io.req.bits.addr_start := ld_bias_addr_start
  ld_bias.io.req.bits.dram_addr := loop_requesting_ld_bias.bias_dram_addr
  ld_bias.io.req.bits.no_bias := loop_requesting_ld_bias.no_bias
  ld_bias.io.req.bits.loop_id := loop_requesting_ld_bias_id

  ld_bias.io.req.valid := !loop_requesting_ld_bias.ld_bias_started && loop_requesting_ld_bias.configured

  when (ld_bias.io.req.fire) {
    loop_requesting_ld_bias.running := true.B
    loop_requesting_ld_bias.ld_bias_started := true.B

    // when (loop_requesting_ld_bias.bias_dram_addr =/= 0.U) {
    when (loop_requesting_ld_bias.output_dram_addr =/= 0.U) {
      ld_bias_addr_start := floorAdd(ld_bias_addr_start, (max_acc_addr / concurrent_loops).U, max_acc_addr.U)
    }
  }

  val loop_requesting_ld_input_id = Mux(head_loop.ld_input_started, tail_loop_id, head_loop_id)
  val loop_requesting_ld_input = loops(loop_requesting_ld_input_id)
  ld_input.io.req.bits.outer_bounds := loop_requesting_ld_input.outer_bounds
  ld_input.io.req.bits.inner_bounds := loop_requesting_ld_input.inner_bounds
  ld_input.io.req.bits.derived_params := loop_requesting_ld_input.derived_params()
  ld_input.io.req.bits.addr_start := loop_requesting_ld_input.a_addr_start
  ld_input.io.req.bits.dram_addr := loop_requesting_ld_input.input_dram_addr
  ld_input.io.req.bits.loop_id := loop_requesting_ld_input_id

  ld_input.io.req.valid := !loop_requesting_ld_input.ld_input_started && loop_requesting_ld_input.configured

  when (ld_input.io.req.fire) {
    loop_requesting_ld_input.running := true.B
    loop_requesting_ld_input.ld_input_started := true.B
  }

  val loop_requesting_ld_weights_id = Mux(head_loop.ld_weights_started, tail_loop_id, head_loop_id)
  val loop_requesting_ld_weights = loops(loop_requesting_ld_weights_id)
  ld_weights.io.req.bits.outer_bounds := loop_requesting_ld_weights.outer_bounds
  ld_weights.io.req.bits.inner_bounds := loop_requesting_ld_weights.inner_bounds
  ld_weights.io.req.bits.derived_params := loop_requesting_ld_weights.derived_params()
  ld_weights.io.req.bits.addr_end := loop_requesting_ld_weights.b_addr_end
  ld_weights.io.req.bits.dram_addr := loop_requesting_ld_weights.weights_dram_addr
  ld_weights.io.req.bits.loop_id := loop_requesting_ld_weights_id

  ld_weights.io.req.valid := !loop_requesting_ld_weights.ld_weights_started && loop_requesting_ld_weights.configured

  when (ld_weights.io.req.fire) {
    loop_requesting_ld_weights.running := true.B
    loop_requesting_ld_weights.ld_weights_started := true.B
  }

  val loop_requesting_ex_id = Mux(head_loop.ex_started, tail_loop_id, head_loop_id)
  val loop_requesting_ex = loops(loop_requesting_ex_id)
  ex.io.req.bits.outer_bounds := loop_requesting_ex.outer_bounds
  ex.io.req.bits.inner_bounds := loop_requesting_ex.inner_bounds
  ex.io.req.bits.derived_params := loop_requesting_ex.derived_params()
  ex.io.req.bits.a_addr_start := loop_requesting_ex.a_addr_start
  ex.io.req.bits.b_addr_end := loop_requesting_ex.b_addr_end
  ex.io.req.bits.c_addr_start := ex_c_addr_start
  ex.io.req.bits.loop_id := loop_requesting_ex_id

  ex.io.req.valid := !loop_requesting_ex.ex_started && loop_requesting_ex.ld_bias_started &&
    loop_requesting_ex.ld_input_started && loop_requesting_ex.ld_weights_started && loop_requesting_ex.configured

  when (ex.io.req.fire) {
    loop_requesting_ex.running := true.B
    loop_requesting_ex.ex_started := true.B

    when (loop_requesting_ex.output_dram_addr =/= 0.U) {
      ex_c_addr_start := floorAdd(ex_c_addr_start, (max_acc_addr / concurrent_loops).U, max_acc_addr.U)
    }
  }

  val loop_requesting_st_id = Mux(head_loop.st_started, tail_loop_id, head_loop_id)
  val loop_requesting_st = loops(loop_requesting_st_id)
  st.io.req.bits.outer_bounds := loop_requesting_st.outer_bounds
  st.io.req.bits.inner_bounds := loop_requesting_st.inner_bounds
  st.io.req.bits.derived_params := loop_requesting_st.derived_params()
  st.io.req.bits.addr_start := st_addr_start
  st.io.req.bits.dram_addr := loop_requesting_st.output_dram_addr
  st.io.req.bits.activation := loop_requesting_st.activation
  st.io.req.bits.loop_id := loop_requesting_st_id

  st.io.req.valid := !loop_requesting_st.st_started && loop_requesting_st.ex_started && loop_requesting_st.configured

  when (st.io.req.fire) {
    loop_requesting_st.running := true.B
    loop_requesting_st.st_started := true.B

    when (loop_requesting_st.output_dram_addr =/= 0.U) {
      st_addr_start := floorAdd(st_addr_start, (max_acc_addr / concurrent_loops).U, max_acc_addr.U)
    }
  }

  // Handle completed signals
  when (ld_bias.io.idle && loops(ld_bias.io.loop_id).running && loops(ld_bias.io.loop_id).ld_bias_started) {
    loops(ld_bias.io.loop_id).ld_bias_completed := true.B
  }

  when (ld_input.io.idle && loops(ld_input.io.loop_id).running && loops(ld_input.io.loop_id).ld_input_started) {
    loops(ld_input.io.loop_id).ld_input_completed := true.B
  }

  when (ld_weights.io.idle && loops(ld_weights.io.loop_id).running && loops(ld_weights.io.loop_id).ld_weights_started) {
    loops(ld_weights.io.loop_id).ld_weights_completed := true.B
  }

  when (ex.io.idle && loops(ex.io.loop_id).running && loops(ex.io.loop_id).ex_started) {
    loops(ex.io.loop_id).ex_completed := true.B
  }

  when (st.io.idle && loops(st.io.loop_id).running && loops(st.io.loop_id).st_started) {
    loops(st.io.loop_id).st_completed := true.B
  }

  when (head_loop.running && head_loop.all_completed()) {
    head_loop.reset()
    head_loop_id := ~head_loop_id
  }

  // Resets
  when (reset.asBool()) {
    loops.zipWithIndex.foreach { case (l, i) =>
      l.reset()
      l.a_addr_start := (i * (max_addr / concurrent_loops)).U
      l.b_addr_end := ((i+1) * (max_addr / concurrent_loops)).U
    }
  }
}

object LoopConvDW {
  def apply(in: DecoupledIO[GemminiCmd], ld_completed: UInt, st_completed: UInt, ex_completed: UInt,
            block_size: Int, coreMaxAddrBits: Int, rob_size: Int, max_lds: Int, max_exs: Int, max_sts: Int,
            max_addr: Int, max_acc_addr: Int, input_w: Int, acc_w: Int, dma_max_bytes: Int,
            config_mvin_rs1_t: ConfigMvinRs1, mvin_rs2_t: MvinRs2, config_mvout_rs2_t: ConfigMvoutRs2,
            mvout_rs2_t: MvoutRs2, config_ex_rs1_t: ConfigExRs1, preload_rs1_t: PreloadRs, preload_rs2_t: PreloadRs,
            compute_rs1_t: ComputeRs, compute_rs2_t: ComputeRs)
           (implicit p: Parameters): (DecoupledIO[GemminiCmd], Bool) = {

    val mod = Module(new LoopConvDW(block_size, coreMaxAddrBits, rob_size, max_lds, max_exs, max_sts,
      max_addr, max_acc_addr, input_w, acc_w, dma_max_bytes,
      config_mvin_rs1_t, mvin_rs2_t, config_mvout_rs2_t, mvout_rs2_t, config_ex_rs1_t, preload_rs1_t, preload_rs2_t,
      compute_rs1_t, compute_rs2_t))

    mod.io.in <> in
    mod.io.ld_completed := ld_completed
    mod.io.st_completed := st_completed
    mod.io.ex_completed := ex_completed
    (mod.io.out, mod.io.busy)
  }

  def castDramOffset(dram_offset: UInt): UInt = {
    // Cast dram offsets to 32 bits max
    dram_offset & "hFFFFFFFF".U
  }
}
