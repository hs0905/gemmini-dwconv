// See README.md for license details.
package gemmini

import chisel3._
import chisel3.util._
import Util._

class PEControl[T <: Data : Arithmetic](accType: T) extends Bundle {  // act like structure in verilog
  val dataflow          = UInt(1.W)                                   // declare dataflow as 1 bit unsigned integer  
  val propagate         = UInt(1.W)                                   // declare propagate as 1 bit unsigned integer                    
  //========= newly added for depthwise(BiRD)=========
  val dwconv_depthwise  = Bool()
  //=================================================
  val shift             = UInt(log2Up(accType.getWidth).W)            // declare shift as log2Up(accType.getWidth).W unsigned integer
}

// TODO update documentation
/**
  * A PE implementing a MAC operation. Configured as fully combinational when integrated into a Mesh.
  * @param width Data width of operands
  */
class PE[T <: Data](inputType: T, outputType: T, accType: T, df: Dataflow.Value, max_simultaneous_matmuls: Int)
                   (implicit ev: Arithmetic[T]) extends Module { // Debugging variables
  import ev._
  val io = IO(new Bundle {
    val in_a                = Input (inputType)   // input feature map
    val in_b                = Input (outputType)  // weight
    val in_d                = Input (outputType)  // next input of input feature map
    val out_a               = Output(inputType)
    val out_b               = Output(outputType)
    val out_c               = Output(outputType)
    val in_control          = Input (new PEControl(accType))
    val out_control         = Output(new PEControl(accType))
    val in_id               = Input (UInt(log2Up(max_simultaneous_matmuls).W))
    val out_id              = Output(UInt(log2Up(max_simultaneous_matmuls).W))
    val in_last             = Input (Bool())
    val out_last            = Output(Bool())
    val in_valid            = Input (Bool())
    val out_valid           = Output(Bool())
    val bad_dataflow        = Output(Bool())
    //========= newly added for depthwise(BiRD)=========
    val in_depthwise_accum  = Input (outputType)
    val out_depthwise_accum = Output(outputType)
    //=================================================
  })

  val cType = if (df == Dataflow.WS) inputType else accType

  val a                 = io.in_a
  val b                 = io.in_b
  val d                 = io.in_d
  val c1                = Reg(cType)
  val c2                = Reg(cType)
  //========= newly added for depthwise(BiRD)=========
  val accum             = Reg(outputType)
  //=================================================
  val dataflow          = io.in_control.dataflow
  val prop              = io.in_control.propagate
  val shift             = io.in_control.shift
  val dwconv_depthwise  = io.in_control.dwconv_depthwise
  val id                = io.in_id
  val last              = io.in_last
  val valid             = io.in_valid

  io.out_a                        := a
  io.out_control.dataflow         := dataflow
  io.out_control.propagate        := prop
  io.out_control.shift            := shift
  //========= newly added for depthwise(BiRD)=========
  io.out_control.dwconv_depthwise := dwconv_depthwise
  //=================================================    
  io.out_id                       := id
  io.out_last                     := last
  io.out_valid                    := valid

  val last_s                      = RegEnable(prop, valid) 
  val flip                        = last_s =/= prop
  val shift_offset                = Mux(flip, shift, 0.U)

  // Which dataflow are we using?
  val OUTPUT_STATIONARY = Dataflow.OS.id.U(1.W) // 0
  val WEIGHT_STATIONARY = Dataflow.WS.id.U(1.W) // 1

  // Is c1 being computed on, or propagated forward (in the output-stationary dataflow)?
  val COMPUTE         = 0.U(1.W)
  val PROPAGATE       = 1.U(1.W)

  io.bad_dataflow := false.B
  //========= newly added for depthwise(BiRD)=========  
  io.out_depthwise_accum := 0.S // default value for depthwise accum
 
  when(dwconv_depthwise){           // if depthwise, then accum is used
    accum := io.in_depthwise_accum  // accum is used for depthwise accum
  }.otherwise {                     // if not depthwise, then c1 is used               
    accum := b                   
  }
  // ====================================================

  // df means dataflow
  // first case is when dataflow is output stationary
  when ((df == Dataflow.OS).B || ((df == Dataflow.BOTH).B && dataflow === OUTPUT_STATIONARY)) {
    when(prop === PROPAGATE) {
      io.out_c  := (c1 >> shift_offset).clippedToWidthOf(outputType)
      io.out_b  := b
      c2        := c2.mac(a, b.asTypeOf(inputType))
      c1        := d.withWidthOf(cType)
    }.otherwise {
      io.out_c  := (c2 >> shift_offset).clippedToWidthOf(outputType)
      io.out_b  := b
      c1        := c1.mac(a, b.asTypeOf(inputType))
      c2        := d.withWidthOf(cType)
    }
  }.elsewhen ((df == Dataflow.WS).B || ((df == Dataflow.BOTH).B && dataflow === WEIGHT_STATIONARY)) // second case is when dataflow is weight stationary
   {
    when(prop === PROPAGATE) { 
      io.out_c                := c1
      io.out_b                := accum.mac(a, c2.asTypeOf(inputType)) // Modified for depthwise(BiRD)
      c1                      := d
      io.out_depthwise_accum  := io.out_b                // Modified for depthwise(BiRD)
    }.otherwise { // prop === COMPUTE
      io.out_c                := c2
      io.out_b                := accum.mac(a, c1.asTypeOf(inputType)) // Modified for depthwise(BiRD)
      c2                      := d
      io.out_depthwise_accum  := io.out_b                // Modified for depthwise(BiRD)
    }
  }.otherwise { // third case is when dataflow is unknown
    io.bad_dataflow         := true.B
    io.out_c                := DontCare
    io.out_b                := DontCare
    io.out_depthwise_accum  := DontCare                 // Modified for depthwise(BiRD)
  }
  when (!valid) {
    c1 := c1
    c2 := c2
  }
}
