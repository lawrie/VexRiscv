// Generator : SpinalHDL v1.3.1    git head : 9fe87c98746a5306cb1d5a828db7af3137723649
// Date      : 02/04/2019, 19:48:00
// Component : MuraxArduino


`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10
`define AluBitwiseCtrlEnum_defaultEncoding_SRC1 2'b11

`define EnvCtrlEnum_defaultEncoding_type [0:0]
`define EnvCtrlEnum_defaultEncoding_NONE 1'b0
`define EnvCtrlEnum_defaultEncoding_XRET 1'b1

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define JtagState_defaultEncoding_type [3:0]
`define JtagState_defaultEncoding_RESET 4'b0000
`define JtagState_defaultEncoding_IDLE 4'b0001
`define JtagState_defaultEncoding_IR_SELECT 4'b0010
`define JtagState_defaultEncoding_IR_CAPTURE 4'b0011
`define JtagState_defaultEncoding_IR_SHIFT 4'b0100
`define JtagState_defaultEncoding_IR_EXIT1 4'b0101
`define JtagState_defaultEncoding_IR_PAUSE 4'b0110
`define JtagState_defaultEncoding_IR_EXIT2 4'b0111
`define JtagState_defaultEncoding_IR_UPDATE 4'b1000
`define JtagState_defaultEncoding_DR_SELECT 4'b1001
`define JtagState_defaultEncoding_DR_CAPTURE 4'b1010
`define JtagState_defaultEncoding_DR_SHIFT 4'b1011
`define JtagState_defaultEncoding_DR_EXIT1 4'b1100
`define JtagState_defaultEncoding_DR_PAUSE 4'b1101
`define JtagState_defaultEncoding_DR_EXIT2 4'b1110
`define JtagState_defaultEncoding_DR_UPDATE 4'b1111

`define UartStopType_defaultEncoding_type [0:0]
`define UartStopType_defaultEncoding_ONE 1'b0
`define UartStopType_defaultEncoding_TWO 1'b1

`define UartParityType_defaultEncoding_type [1:0]
`define UartParityType_defaultEncoding_NONE 2'b00
`define UartParityType_defaultEncoding_EVEN 2'b01
`define UartParityType_defaultEncoding_ODD 2'b10

`define UartCtrlTxState_defaultEncoding_type [2:0]
`define UartCtrlTxState_defaultEncoding_IDLE 3'b000
`define UartCtrlTxState_defaultEncoding_START 3'b001
`define UartCtrlTxState_defaultEncoding_DATA 3'b010
`define UartCtrlTxState_defaultEncoding_PARITY 3'b011
`define UartCtrlTxState_defaultEncoding_STOP 3'b100

`define UartCtrlRxState_defaultEncoding_type [2:0]
`define UartCtrlRxState_defaultEncoding_IDLE 3'b000
`define UartCtrlRxState_defaultEncoding_START 3'b001
`define UartCtrlRxState_defaultEncoding_DATA 3'b010
`define UartCtrlRxState_defaultEncoding_PARITY 3'b011
`define UartCtrlRxState_defaultEncoding_STOP 3'b100

`define SpiMasterCtrlCmdMode_defaultEncoding_type [0:0]
`define SpiMasterCtrlCmdMode_defaultEncoding_DATA 1'b0
`define SpiMasterCtrlCmdMode_defaultEncoding_SS 1'b1

`define I2cSlaveCmdMode_defaultEncoding_type [2:0]
`define I2cSlaveCmdMode_defaultEncoding_NONE 3'b000
`define I2cSlaveCmdMode_defaultEncoding_START 3'b001
`define I2cSlaveCmdMode_defaultEncoding_RESTART 3'b010
`define I2cSlaveCmdMode_defaultEncoding_STOP 3'b011
`define I2cSlaveCmdMode_defaultEncoding_DROP 3'b100
`define I2cSlaveCmdMode_defaultEncoding_DRIVE 3'b101
`define I2cSlaveCmdMode_defaultEncoding_READ 3'b110

`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_type [3:0]
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_boot 4'b0000
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE 4'b0001
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START 4'b0010
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW 4'b0011
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH 4'b0100
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART 4'b0101
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1 4'b0110
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2 4'b0111
`define bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF 4'b1000

module BufferCC (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_1_ (
      input   io_dataIn,
      output  io_dataOut,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_mainClkReset);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_io_mainClk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module UartCtrlTx (
      input  [2:0] io_configFrame_dataLength,
      input  `UartStopType_defaultEncoding_type io_configFrame_stop,
      input  `UartParityType_defaultEncoding_type io_configFrame_parity,
      input   io_samplingTick,
      input   io_write_valid,
      output reg  io_write_ready,
      input  [7:0] io_write_payload,
      output  io_txd,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_1_;
  wire [0:0] _zz_2_;
  wire [2:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [2:0] _zz_5_;
  reg  clockDivider_counter_willIncrement;
  wire  clockDivider_counter_willClear;
  reg [2:0] clockDivider_counter_valueNext;
  reg [2:0] clockDivider_counter_value;
  wire  clockDivider_counter_willOverflowIfInc;
  wire  clockDivider_willOverflow;
  reg [2:0] tickCounter_value;
  reg `UartCtrlTxState_defaultEncoding_type stateMachine_state;
  reg  stateMachine_parity;
  reg  stateMachine_txd;
  reg  stateMachine_txd_regNext;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif

  assign _zz_1_ = (tickCounter_value == io_configFrame_dataLength);
  assign _zz_2_ = clockDivider_counter_willIncrement;
  assign _zz_3_ = {2'd0, _zz_2_};
  assign _zz_4_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_5_ = {2'd0, _zz_4_};
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlTxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlTxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlTxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlTxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    clockDivider_counter_willIncrement = 1'b0;
    if(io_samplingTick)begin
      clockDivider_counter_willIncrement = 1'b1;
    end
  end

  assign clockDivider_counter_willClear = 1'b0;
  assign clockDivider_counter_willOverflowIfInc = (clockDivider_counter_value == (3'b100));
  assign clockDivider_willOverflow = (clockDivider_counter_willOverflowIfInc && clockDivider_counter_willIncrement);
  always @ (*) begin
    if(clockDivider_willOverflow)begin
      clockDivider_counter_valueNext = (3'b000);
    end else begin
      clockDivider_counter_valueNext = (clockDivider_counter_value + _zz_3_);
    end
    if(clockDivider_counter_willClear)begin
      clockDivider_counter_valueNext = (3'b000);
    end
  end

  always @ (*) begin
    stateMachine_txd = 1'b1;
    io_write_ready = 1'b0;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        stateMachine_txd = 1'b0;
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        stateMachine_txd = io_write_payload[tickCounter_value];
        if(clockDivider_willOverflow)begin
          if(_zz_1_)begin
            io_write_ready = 1'b1;
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        stateMachine_txd = stateMachine_parity;
      end
      default : begin
      end
    endcase
  end

  assign io_txd = stateMachine_txd_regNext;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      clockDivider_counter_value <= (3'b000);
      stateMachine_state <= `UartCtrlTxState_defaultEncoding_IDLE;
      stateMachine_txd_regNext <= 1'b1;
    end else begin
      clockDivider_counter_value <= clockDivider_counter_valueNext;
      case(stateMachine_state)
        `UartCtrlTxState_defaultEncoding_IDLE : begin
          if((io_write_valid && clockDivider_willOverflow))begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_START;
          end
        end
        `UartCtrlTxState_defaultEncoding_START : begin
          if(clockDivider_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_DATA;
          end
        end
        `UartCtrlTxState_defaultEncoding_DATA : begin
          if(clockDivider_willOverflow)begin
            if(_zz_1_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
              end else begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlTxState_defaultEncoding_PARITY : begin
          if(clockDivider_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
          end
        end
        default : begin
          if(clockDivider_willOverflow)begin
            if((tickCounter_value == _zz_5_))begin
              stateMachine_state <= (io_write_valid ? `UartCtrlTxState_defaultEncoding_START : `UartCtrlTxState_defaultEncoding_IDLE);
            end
          end
        end
      endcase
      stateMachine_txd_regNext <= stateMachine_txd;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(clockDivider_willOverflow)begin
      tickCounter_value <= (tickCounter_value + (3'b001));
    end
    if(clockDivider_willOverflow)begin
      stateMachine_parity <= (stateMachine_parity ^ stateMachine_txd);
    end
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        if(clockDivider_willOverflow)begin
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
          tickCounter_value <= (3'b000);
        end
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_willOverflow)begin
          if(_zz_1_)begin
            tickCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        if(clockDivider_willOverflow)begin
          tickCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module UartCtrlRx (
      input  [2:0] io_configFrame_dataLength,
      input  `UartStopType_defaultEncoding_type io_configFrame_stop,
      input  `UartParityType_defaultEncoding_type io_configFrame_parity,
      input   io_samplingTick,
      output  io_read_valid,
      output [7:0] io_read_payload,
      input   io_rxd,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_1_;
  wire  bufferCC_5__io_dataOut;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire [0:0] _zz_5_;
  wire [2:0] _zz_6_;
  wire  sampler_synchroniser;
  wire  sampler_samples_0;
  reg  sampler_samples_1;
  reg  sampler_samples_2;
  reg  sampler_value;
  reg  sampler_tick;
  reg [2:0] bitTimer_counter;
  reg  bitTimer_tick;
  reg [2:0] bitCounter_value;
  reg `UartCtrlRxState_defaultEncoding_type stateMachine_state;
  reg  stateMachine_parity;
  reg [7:0] stateMachine_shifter;
  reg  stateMachine_validReg;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif

  assign _zz_2_ = (bitTimer_counter == (3'b000));
  assign _zz_3_ = (sampler_tick && (! sampler_value));
  assign _zz_4_ = (bitCounter_value == io_configFrame_dataLength);
  assign _zz_5_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_6_ = {2'd0, _zz_5_};
  BufferCC bufferCC_5_ ( 
    .io_initial(_zz_1_),
    .io_dataIn(io_rxd),
    .io_dataOut(bufferCC_5__io_dataOut),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlRxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlRxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlRxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlRxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  assign _zz_1_ = 1'b0;
  assign sampler_synchroniser = bufferCC_5__io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @ (*) begin
    bitTimer_tick = 1'b0;
    if(sampler_tick)begin
      if(_zz_2_)begin
        bitTimer_tick = 1'b1;
      end
    end
  end

  assign io_read_valid = stateMachine_validReg;
  assign io_read_payload = stateMachine_shifter;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b0;
      stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
      stateMachine_validReg <= 1'b0;
    end else begin
      if(io_samplingTick)begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(io_samplingTick)begin
        sampler_samples_2 <= sampler_samples_1;
      end
      sampler_value <= (((1'b0 || ((1'b1 && sampler_samples_0) && sampler_samples_1)) || ((1'b1 && sampler_samples_0) && sampler_samples_2)) || ((1'b1 && sampler_samples_1) && sampler_samples_2));
      sampler_tick <= io_samplingTick;
      stateMachine_validReg <= 1'b0;
      case(stateMachine_state)
        `UartCtrlRxState_defaultEncoding_IDLE : begin
          if(_zz_3_)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_START;
          end
        end
        `UartCtrlRxState_defaultEncoding_START : begin
          if(bitTimer_tick)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_DATA;
            if((sampler_value == 1'b1))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_DATA : begin
          if(bitTimer_tick)begin
            if(_zz_4_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
                stateMachine_validReg <= 1'b1;
              end else begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_PARITY : begin
          if(bitTimer_tick)begin
            if((stateMachine_parity == sampler_value))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
              stateMachine_validReg <= 1'b1;
            end else begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        default : begin
          if(bitTimer_tick)begin
            if((! sampler_value))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end else begin
              if((bitCounter_value == _zz_6_))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
              end
            end
          end
        end
      endcase
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(sampler_tick)begin
      bitTimer_counter <= (bitTimer_counter - (3'b001));
      if(_zz_2_)begin
        bitTimer_counter <= (3'b100);
      end
    end
    if(bitTimer_tick)begin
      bitCounter_value <= (bitCounter_value + (3'b001));
    end
    if(bitTimer_tick)begin
      stateMachine_parity <= (stateMachine_parity ^ sampler_value);
    end
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
        if(_zz_3_)begin
          bitTimer_counter <= (3'b001);
        end
      end
      `UartCtrlRxState_defaultEncoding_START : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
        end
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
        if(bitTimer_tick)begin
          stateMachine_shifter[bitCounter_value] <= sampler_value;
          if(_zz_4_)begin
            bitCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end

endmodule


//BufferCC_2_ remplaced by BufferCC


//BufferCC_3_ remplaced by BufferCC

module QspiSlaveRX (
      input   io_qckRise,
      input   io_qssRise,
      input  [3:0] io_qd,
      output  io_rxReady,
      output [7:0] io_rxData,
      output [3:0] io_byteNumber,
      input   io_readEnable,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [7:0] shiftReg;
  reg [7:0] lastByte;
  reg [3:0] byteNumber;
  reg  firstNibble;
  reg  rxReady;
  assign io_rxData = shiftReg;
  assign io_byteNumber = byteNumber;
  assign io_rxReady = rxReady;
  always @ (posedge toplevel_io_mainClk) begin
    rxReady <= 1'b0;
    if(io_readEnable)begin
      if(io_qssRise)begin
        firstNibble <= 1'b1;
        lastByte <= shiftReg;
      end
      if(io_qckRise)begin
        if(firstNibble)begin
          byteNumber <= (byteNumber + (4'b0001));
          shiftReg[7 : 4] <= io_qd;
          firstNibble <= 1'b0;
        end else begin
          shiftReg[3 : 0] <= io_qd;
          rxReady <= 1'b1;
        end
      end
    end
  end

endmodule

module QspiSlaveTX (
      input   io_qckFall,
      input   io_qssRise,
      output [3:0] io_qd,
      output  io_txReady,
      input  [7:0] io_txData,
      input   io_writeEnable,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [3:0] outData;
  reg  firstNibble;
  reg  txReady;
  assign io_qd = outData;
  assign io_txReady = txReady;
  always @ (posedge toplevel_io_mainClk) begin
    if(io_writeEnable)begin
      if(io_qssRise)begin
        firstNibble <= 1'b1;
      end
      if(io_qckFall)begin
        if(firstNibble)begin
          outData <= io_txData[7 : 4];
          firstNibble <= 1'b0;
        end else begin
          outData <= io_txData[3 : 0];
          txReady <= 1'b1;
        end
      end
    end
  end

endmodule

module StreamFifoLowLatency (
      input   io_push_valid,
      output  io_push_ready,
      input   io_push_payload_error,
      input  [31:0] io_push_payload_inst,
      output reg  io_pop_valid,
      input   io_pop_ready,
      output reg  io_pop_payload_error,
      output reg [31:0] io_pop_payload_inst,
      input   io_flush,
      output [0:0] io_occupancy,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [0:0] _zz_5_;
  reg  _zz_1_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [32:0] _zz_2_;
  wire [32:0] _zz_3_;
  reg [32:0] _zz_4_;
  assign _zz_5_ = _zz_2_[0 : 0];
  always @ (*) begin
    _zz_1_ = 1'b0;
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      _zz_1_ = 1'b1;
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = 1'b1;
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = 1'b1;
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  assign ptrMatch = 1'b1;
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
      io_pop_payload_error = _zz_5_[0];
      io_pop_payload_inst = _zz_2_[32 : 1];
    end else begin
      io_pop_valid = io_push_valid;
      io_pop_payload_error = io_push_payload_error;
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign _zz_2_ = _zz_3_;
  assign io_occupancy = (risingOccupancy && ptrMatch);
  assign _zz_3_ = _zz_4_;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      risingOccupancy <= 1'b0;
    end else begin
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_1_)begin
      _zz_4_ <= {io_push_payload_inst,io_push_payload_error};
    end
  end

endmodule

module FlowCCByToggle (
      input   io_input_valid,
      input   io_input_payload_last,
      input  [0:0] io_input_payload_fragment,
      output  io_output_valid,
      output  io_output_payload_last,
      output [0:0] io_output_payload_fragment,
      input   _zz_1_,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_mainClkReset);
  wire  bufferCC_5__io_dataOut;
  wire  outHitSignal;
  reg  inputArea_target = 0;
  reg  inputArea_data_last;
  reg [0:0] inputArea_data_fragment;
  wire  outputArea_target;
  reg  outputArea_hit;
  wire  outputArea_flow_valid;
  wire  outputArea_flow_payload_last;
  wire [0:0] outputArea_flow_payload_fragment;
  reg  outputArea_flow_m2sPipe_valid;
  reg  outputArea_flow_m2sPipe_payload_last;
  reg [0:0] outputArea_flow_m2sPipe_payload_fragment;
  BufferCC_1_ bufferCC_5_ ( 
    .io_dataIn(inputArea_target),
    .io_dataOut(bufferCC_5__io_dataOut),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_mainClkReset(toplevel_resetCtrl_mainClkReset) 
  );
  assign outputArea_target = bufferCC_5__io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_last = inputArea_data_last;
  assign outputArea_flow_payload_fragment = inputArea_data_fragment;
  assign io_output_valid = outputArea_flow_m2sPipe_valid;
  assign io_output_payload_last = outputArea_flow_m2sPipe_payload_last;
  assign io_output_payload_fragment = outputArea_flow_m2sPipe_payload_fragment;
  always @ (posedge _zz_1_) begin
    if(io_input_valid)begin
      inputArea_target <= (! inputArea_target);
      inputArea_data_last <= io_input_payload_last;
      inputArea_data_fragment <= io_input_payload_fragment;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    outputArea_hit <= outputArea_target;
    if(outputArea_flow_valid)begin
      outputArea_flow_m2sPipe_payload_last <= outputArea_flow_payload_last;
      outputArea_flow_m2sPipe_payload_fragment <= outputArea_flow_payload_fragment;
    end
  end

  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_mainClkReset) begin
    if (toplevel_resetCtrl_mainClkReset) begin
      outputArea_flow_m2sPipe_valid <= 1'b0;
    end else begin
      outputArea_flow_m2sPipe_valid <= outputArea_flow_valid;
    end
  end

endmodule

module UartCtrl (
      input  [2:0] io_config_frame_dataLength,
      input  `UartStopType_defaultEncoding_type io_config_frame_stop,
      input  `UartParityType_defaultEncoding_type io_config_frame_parity,
      input  [19:0] io_config_clockDivider,
      input   io_write_valid,
      output  io_write_ready,
      input  [7:0] io_write_payload,
      output  io_read_valid,
      output [7:0] io_read_payload,
      output  io_uart_txd,
      input   io_uart_rxd,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  tx_io_write_ready;
  wire  tx_io_txd;
  wire  rx_io_read_valid;
  wire [7:0] rx_io_read_payload;
  reg [19:0] clockDivider_counter;
  wire  clockDivider_tick;
  `ifndef SYNTHESIS
  reg [23:0] io_config_frame_stop_string;
  reg [31:0] io_config_frame_parity_string;
  `endif

  UartCtrlTx tx ( 
    .io_configFrame_dataLength(io_config_frame_dataLength),
    .io_configFrame_stop(io_config_frame_stop),
    .io_configFrame_parity(io_config_frame_parity),
    .io_samplingTick(clockDivider_tick),
    .io_write_valid(io_write_valid),
    .io_write_ready(tx_io_write_ready),
    .io_write_payload(io_write_payload),
    .io_txd(tx_io_txd),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  UartCtrlRx rx ( 
    .io_configFrame_dataLength(io_config_frame_dataLength),
    .io_configFrame_stop(io_config_frame_stop),
    .io_configFrame_parity(io_config_frame_parity),
    .io_samplingTick(clockDivider_tick),
    .io_read_valid(rx_io_read_valid),
    .io_read_payload(rx_io_read_payload),
    .io_rxd(io_uart_rxd),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_config_frame_stop)
      `UartStopType_defaultEncoding_ONE : io_config_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_config_frame_stop_string = "TWO";
      default : io_config_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_config_frame_parity)
      `UartParityType_defaultEncoding_NONE : io_config_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_config_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_config_frame_parity_string = "ODD ";
      default : io_config_frame_parity_string = "????";
    endcase
  end
  `endif

  assign clockDivider_tick = (clockDivider_counter == (20'b00000000000000000000));
  assign io_write_ready = tx_io_write_ready;
  assign io_read_valid = rx_io_read_valid;
  assign io_read_payload = rx_io_read_payload;
  assign io_uart_txd = tx_io_txd;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      clockDivider_counter <= (20'b00000000000000000000);
    end else begin
      clockDivider_counter <= (clockDivider_counter - (20'b00000000000000000001));
      if(clockDivider_tick)begin
        clockDivider_counter <= io_config_clockDivider;
      end
    end
  end

endmodule

module StreamFifo (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      input   io_flush,
      output [4:0] io_occupancy,
      output [4:0] io_availability,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [7:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [3:0] _zz_5_;
  wire [0:0] _zz_6_;
  wire [3:0] _zz_7_;
  wire [3:0] _zz_8_;
  wire  _zz_9_;
  reg  _zz_1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [3:0] logic_pushPtr_valueNext;
  reg [3:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [3:0] logic_popPtr_valueNext;
  reg [3:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_2_;
  wire [3:0] logic_ptrDif;
  reg [7:0] logic_ram [0:15];
  assign _zz_4_ = logic_pushPtr_willIncrement;
  assign _zz_5_ = {3'd0, _zz_4_};
  assign _zz_6_ = logic_popPtr_willIncrement;
  assign _zz_7_ = {3'd0, _zz_6_};
  assign _zz_8_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_9_ = 1'b1;
  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_1_) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_9_) begin
      _zz_3_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      _zz_1_ = 1'b1;
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (4'b1111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_5_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (4'b0000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (4'b1111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_7_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (4'b0000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2_ && (! logic_full))));
  assign io_pop_payload = _zz_3_;
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_8_};
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      logic_pushPtr_value <= (4'b0000);
      logic_popPtr_value <= (4'b0000);
      logic_risingOccupancy <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule


//StreamFifo_1_ remplaced by StreamFifo

module PinInterruptCtrl (
      input  [1:0] io_pinInterrupt_pins,
      input  [1:0] io_rising,
      input  [1:0] io_falling,
      output reg [1:0] io_interrupt,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_1_;
  reg  _zz_1__regNext;
  wire  _zz_2_;
  reg  _zz_2__regNext;
  wire  _zz_3_;
  reg  _zz_3__regNext;
  wire  _zz_4_;
  reg  _zz_4__regNext;
  always @ (*) begin
    io_interrupt[0] = 1'b0;
    if(((io_rising[0] && (_zz_1_ && (! _zz_1__regNext))) || (io_falling[0] && ((! _zz_2_) && _zz_2__regNext))))begin
      io_interrupt[0] = 1'b1;
    end
    io_interrupt[1] = 1'b0;
    if(((io_rising[1] && (_zz_3_ && (! _zz_3__regNext))) || (io_falling[1] && ((! _zz_4_) && _zz_4__regNext))))begin
      io_interrupt[1] = 1'b1;
    end
  end

  assign _zz_1_ = io_pinInterrupt_pins[0];
  assign _zz_2_ = io_pinInterrupt_pins[0];
  assign _zz_3_ = io_pinInterrupt_pins[1];
  assign _zz_4_ = io_pinInterrupt_pins[1];
  always @ (posedge toplevel_io_mainClk) begin
    _zz_1__regNext <= _zz_1_;
    _zz_2__regNext <= _zz_2_;
    _zz_3__regNext <= _zz_3_;
    _zz_4__regNext <= _zz_4_;
  end

endmodule

module InterruptCtrl (
      input  [1:0] io_inputs,
      input  [1:0] io_clears,
      input  [1:0] io_masks,
      output [1:0] io_pendings,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [1:0] pendings;
  assign io_pendings = (pendings & io_masks);
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      pendings <= (2'b00);
    end else begin
      pendings <= ((pendings & (~ io_clears)) | io_inputs);
    end
  end

endmodule

module Prescaler (
      input   io_clear,
      input  [15:0] io_limit,
      output  io_overflow,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [15:0] counter;
  assign io_overflow = (counter == io_limit);
  always @ (posedge toplevel_io_mainClk) begin
    counter <= (counter + (16'b0000000000000001));
    if((io_clear || io_overflow))begin
      counter <= (16'b0000000000000000);
    end
  end

endmodule

module Timer (
      input   io_tick,
      input   io_clear,
      input  [15:0] io_limit,
      output  io_full,
      output [15:0] io_value,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [0:0] _zz_1_;
  wire [15:0] _zz_2_;
  reg [15:0] counter;
  wire  limitHit;
  reg  inhibitFull;
  assign _zz_1_ = (! limitHit);
  assign _zz_2_ = {15'd0, _zz_1_};
  assign limitHit = (counter == io_limit);
  assign io_full = ((limitHit && io_tick) && (! inhibitFull));
  assign io_value = counter;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      inhibitFull <= 1'b0;
    end else begin
      if(io_tick)begin
        inhibitFull <= limitHit;
      end
      if(io_clear)begin
        inhibitFull <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(io_tick)begin
      counter <= (counter + _zz_2_);
    end
    if(io_clear)begin
      counter <= (16'b0000000000000000);
    end
  end

endmodule


//Timer_1_ remplaced by Timer


//InterruptCtrl_1_ remplaced by InterruptCtrl

module PwmCtrl (
      output reg [2:0] io_pwm_pins,
      input  [7:0] io_duty_0,
      input  [7:0] io_duty_1,
      input  [7:0] io_duty_2,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [7:0] counter;
  always @ (*) begin
    io_pwm_pins[0] = ((counter <= io_duty_0) && (io_duty_0 != (8'b00000000)));
    io_pwm_pins[1] = ((counter <= io_duty_1) && (io_duty_1 != (8'b00000000)));
    io_pwm_pins[2] = ((counter <= io_duty_2) && (io_duty_2 != (8'b00000000)));
  end

  always @ (posedge toplevel_io_mainClk) begin
    counter <= (counter + (8'b00000001));
  end

endmodule

module ServoCtrl (
      output  io_servo_pin,
      input  [11:0] io_pulseMicros,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [14:0] _zz_1_;
  reg [5:0] counter;
  reg [14:0] micros;
  assign _zz_1_ = {3'd0, io_pulseMicros};
  assign io_servo_pin = ((micros < _zz_1_) && (io_pulseMicros != (12'b000000000000)));
  always @ (posedge toplevel_io_mainClk) begin
    counter <= (counter + (6'b000001));
    if((counter == (6'b110001)))begin
      micros <= (micros + (15'b000000000000001));
      counter <= (6'b000000);
    end
    if((micros == (15'b100111000011111)))begin
      micros <= (15'b000000000000000);
    end
  end

endmodule

module MuxCtrl (
      output [31:0] io_mux_pins,
      input  [31:0] io_signals);
  assign io_mux_pins = io_signals;
endmodule

module MachineTimerCtrl (
      output [31:0] io_micros,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [5:0] counter;
  reg [31:0] microCounter;
  assign io_micros = microCounter;
  always @ (posedge toplevel_io_mainClk) begin
    counter <= (counter + (6'b000001));
    if((counter == (6'b110001)))begin
      counter <= (6'b000000);
      microCounter <= (microCounter + (32'b00000000000000000000000000000001));
    end
  end

endmodule

module ToneCtrl (
      output  io_tone_pin,
      input  [31:0] io_period,
      input  [31:0] io_duration,
      input   io_clear,
      output  io_done,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [31:0] _zz_1_;
  reg [7:0] prescaler_1_;
  reg [31:0] counter;
  reg [31:0] timeCounter;
  reg [31:0] millis;
  reg  toneOut;
  assign _zz_1_ = (io_period - (32'b00000000000000000000000000000001));
  assign io_tone_pin = toneOut;
  assign io_done = (millis == io_duration);
  always @ (posedge toplevel_io_mainClk) begin
    if(io_clear)begin
      millis <= (32'b00000000000000000000000000000000);
    end
    if((! io_done))begin
      if((timeCounter == (32'b00000000000000001100001101001111)))begin
        millis <= (millis + (32'b00000000000000000000000000000001));
        timeCounter <= (32'b00000000000000000000000000000000);
      end else begin
        timeCounter <= (timeCounter + (32'b00000000000000000000000000000001));
      end
      prescaler_1_ <= (prescaler_1_ + (8'b00000001));
      if((prescaler_1_ == (8'b00011000)))begin
        prescaler_1_ <= (8'b00000000);
        counter <= (counter + (32'b00000000000000000000000000000001));
        if((counter == _zz_1_))begin
          counter <= (32'b00000000000000000000000000000000);
          toneOut <= (! toneOut);
        end
      end
    end else begin
      toneOut <= 1'b0;
    end
  end

endmodule

module ShiftOutCtrl (
      output  io_shiftOut_dataPin,
      output  io_shiftOut_clockPin,
      input   io_bitOrder,
      input  [7:0] io_value,
      input  [31:0] io_preScale,
      input   io_set,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [31:0] _zz_1_;
  reg [31:0] prescaler_1_;
  reg [7:0] shiftReg;
  reg [3:0] bitCounter;
  reg  clockReg;
  assign _zz_1_ = (io_preScale - (32'b00000000000000000000000000000001));
  assign io_shiftOut_dataPin = (io_bitOrder ? shiftReg[7] : shiftReg[0]);
  assign io_shiftOut_clockPin = clockReg;
  always @ (posedge toplevel_io_mainClk) begin
    if(io_set)begin
      bitCounter <= (4'b1000);
      clockReg <= 1'b0;
      shiftReg <= io_value;
    end
    if((! (bitCounter == (4'b0000))))begin
      prescaler_1_ <= (prescaler_1_ + (32'b00000000000000000000000000000001));
      if((prescaler_1_ == _zz_1_))begin
        prescaler_1_ <= (32'b00000000000000000000000000000000);
        clockReg <= (! clockReg);
        if(clockReg)begin
          bitCounter <= (bitCounter - (4'b0001));
          if(io_bitOrder)begin
            shiftReg <= (shiftReg <<< 1);
          end else begin
            shiftReg <= (shiftReg >>> 1);
          end
        end
      end
    end
  end

endmodule

module SpiMasterCtrl (
      input   io_config_kind_cpol,
      input   io_config_kind_cpha,
      input  [31:0] io_config_sclkToogle,
      input  [0:0] io_config_ss_activeHigh,
      input  [31:0] io_config_ss_setup,
      input  [31:0] io_config_ss_hold,
      input  [31:0] io_config_ss_disable,
      input   io_cmd_valid,
      output reg  io_cmd_ready,
      input  `SpiMasterCtrlCmdMode_defaultEncoding_type io_cmd_payload_mode,
      input  [8:0] io_cmd_payload_args,
      output  io_rsp_valid,
      output [7:0] io_rsp_payload,
      output [0:0] io_spi_ss,
      output  io_spi_sclk,
      output  io_spi_mosi,
      input   io_spi_miso,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire [0:0] _zz_8_;
  wire [3:0] _zz_9_;
  wire [8:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [0:0] _zz_12_;
  wire [7:0] _zz_13_;
  wire [2:0] _zz_14_;
  wire [2:0] _zz_15_;
  reg [31:0] timer_counter;
  reg  timer_reset;
  wire  timer_ss_setupHit;
  wire  timer_ss_holdHit;
  wire  timer_ss_disableHit;
  wire  timer_sclkToogleHit;
  reg  fsm_counter_willIncrement;
  wire  fsm_counter_willClear;
  reg [3:0] fsm_counter_valueNext;
  reg [3:0] fsm_counter_value;
  wire  fsm_counter_willOverflowIfInc;
  wire  fsm_counter_willOverflow;
  reg [7:0] fsm_buffer;
  reg [0:0] fsm_ss;
  reg  _zz_1_;
  reg  _zz_2_;
  reg  _zz_3_;
  `ifndef SYNTHESIS
  reg [31:0] io_cmd_payload_mode_string;
  `endif

  assign _zz_4_ = (io_cmd_payload_mode == `SpiMasterCtrlCmdMode_defaultEncoding_DATA);
  assign _zz_5_ = _zz_11_[0];
  assign _zz_6_ = (! fsm_counter_value[0]);
  assign _zz_7_ = ((! io_cmd_valid) || io_cmd_ready);
  assign _zz_8_ = fsm_counter_willIncrement;
  assign _zz_9_ = {3'd0, _zz_8_};
  assign _zz_10_ = {fsm_buffer,io_spi_miso};
  assign _zz_11_ = io_cmd_payload_args[0 : 0];
  assign _zz_12_ = io_cmd_payload_args[8 : 8];
  assign _zz_13_ = io_cmd_payload_args[7 : 0];
  assign _zz_14_ = ((3'b111) - _zz_15_);
  assign _zz_15_ = (fsm_counter_value >>> 1);
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_cmd_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : io_cmd_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : io_cmd_payload_mode_string = "SS  ";
      default : io_cmd_payload_mode_string = "????";
    endcase
  end
  `endif

  always @ (*) begin
    timer_reset = 1'b0;
    fsm_counter_willIncrement = 1'b0;
    io_cmd_ready = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_4_)begin
        if(timer_sclkToogleHit)begin
          fsm_counter_willIncrement = 1'b1;
          timer_reset = 1'b1;
          io_cmd_ready = fsm_counter_willOverflowIfInc;
        end
      end else begin
        if(_zz_5_)begin
          if(timer_ss_setupHit)begin
            io_cmd_ready = 1'b1;
          end
        end else begin
          if(_zz_6_)begin
            if(timer_ss_holdHit)begin
              fsm_counter_willIncrement = 1'b1;
              timer_reset = 1'b1;
            end
          end else begin
            if(timer_ss_disableHit)begin
              io_cmd_ready = 1'b1;
            end
          end
        end
      end
    end
    if(_zz_7_)begin
      timer_reset = 1'b1;
    end
  end

  assign timer_ss_setupHit = (timer_counter == io_config_ss_setup);
  assign timer_ss_holdHit = (timer_counter == io_config_ss_hold);
  assign timer_ss_disableHit = (timer_counter == io_config_ss_disable);
  assign timer_sclkToogleHit = (timer_counter == io_config_sclkToogle);
  assign fsm_counter_willClear = 1'b0;
  assign fsm_counter_willOverflowIfInc = (fsm_counter_value == (4'b1111));
  assign fsm_counter_willOverflow = (fsm_counter_willOverflowIfInc && fsm_counter_willIncrement);
  always @ (*) begin
    fsm_counter_valueNext = (fsm_counter_value + _zz_9_);
    if(fsm_counter_willClear)begin
      fsm_counter_valueNext = (4'b0000);
    end
  end

  assign io_rsp_valid = _zz_1_;
  assign io_rsp_payload = fsm_buffer;
  assign io_spi_ss = (fsm_ss ^ io_config_ss_activeHigh);
  assign io_spi_sclk = _zz_2_;
  assign io_spi_mosi = _zz_3_;
  always @ (posedge toplevel_io_mainClk) begin
    timer_counter <= (timer_counter + (32'b00000000000000000000000000000001));
    if(timer_reset)begin
      timer_counter <= (32'b00000000000000000000000000000000);
    end
    if(io_cmd_valid)begin
      if(_zz_4_)begin
        if(timer_sclkToogleHit)begin
          if(fsm_counter_value[0])begin
            fsm_buffer <= _zz_10_[7:0];
          end
        end
      end
    end
    _zz_2_ <= (((io_cmd_valid && (io_cmd_payload_mode == `SpiMasterCtrlCmdMode_defaultEncoding_DATA)) && (fsm_counter_value[0] ^ io_config_kind_cpha)) ^ io_config_kind_cpol);
    _zz_3_ <= _zz_13_[_zz_14_];
  end

  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      fsm_counter_value <= (4'b0000);
      fsm_ss <= (1'b1);
      _zz_1_ <= 1'b0;
    end else begin
      fsm_counter_value <= fsm_counter_valueNext;
      if(io_cmd_valid)begin
        if(! _zz_4_) begin
          if(_zz_5_)begin
            fsm_ss[0] <= 1'b0;
          end else begin
            if(! _zz_6_) begin
              fsm_ss[0] <= 1'b1;
            end
          end
        end
      end
      _zz_1_ <= (((io_cmd_valid && io_cmd_ready) && (io_cmd_payload_mode == `SpiMasterCtrlCmdMode_defaultEncoding_DATA)) && _zz_12_[0]);
      if(_zz_7_)begin
        fsm_counter_value <= (4'b0000);
      end
    end
  end

endmodule

module StreamFifo_2_ (
      input   io_push_valid,
      output  io_push_ready,
      input  `SpiMasterCtrlCmdMode_defaultEncoding_type io_push_payload_mode,
      input  [8:0] io_push_payload_args,
      output  io_pop_valid,
      input   io_pop_ready,
      output `SpiMasterCtrlCmdMode_defaultEncoding_type io_pop_payload_mode,
      output [8:0] io_pop_payload_args,
      input   io_flush,
      output [5:0] io_occupancy,
      output [5:0] io_availability,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [9:0] _zz_6_;
  wire [0:0] _zz_7_;
  wire [4:0] _zz_8_;
  wire [0:0] _zz_9_;
  wire [4:0] _zz_10_;
  wire [4:0] _zz_11_;
  wire  _zz_12_;
  wire [9:0] _zz_13_;
  reg  _zz_1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [4:0] logic_pushPtr_valueNext;
  reg [4:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [4:0] logic_popPtr_valueNext;
  reg [4:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_2_;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type _zz_3_;
  wire [9:0] _zz_4_;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type _zz_5_;
  wire [4:0] logic_ptrDif;
  `ifndef SYNTHESIS
  reg [31:0] io_push_payload_mode_string;
  reg [31:0] io_pop_payload_mode_string;
  reg [31:0] _zz_3__string;
  reg [31:0] _zz_5__string;
  `endif

  reg [9:0] logic_ram [0:31];
  assign _zz_7_ = logic_pushPtr_willIncrement;
  assign _zz_8_ = {4'd0, _zz_7_};
  assign _zz_9_ = logic_popPtr_willIncrement;
  assign _zz_10_ = {4'd0, _zz_9_};
  assign _zz_11_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_12_ = 1'b1;
  assign _zz_13_ = {io_push_payload_args,io_push_payload_mode};
  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_1_) begin
      logic_ram[logic_pushPtr_value] <= _zz_13_;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_12_) begin
      _zz_6_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(io_push_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : io_push_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : io_push_payload_mode_string = "SS  ";
      default : io_push_payload_mode_string = "????";
    endcase
  end
  always @(*) begin
    case(io_pop_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : io_pop_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : io_pop_payload_mode_string = "SS  ";
      default : io_pop_payload_mode_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : _zz_3__string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : _zz_3__string = "SS  ";
      default : _zz_3__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : _zz_5__string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : _zz_5__string = "SS  ";
      default : _zz_5__string = "????";
    endcase
  end
  `endif

  always @ (*) begin
    _zz_1_ = 1'b0;
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      _zz_1_ = 1'b1;
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (5'b11111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_8_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (5'b00000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (5'b11111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_10_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (5'b00000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2_ && (! logic_full))));
  assign _zz_4_ = _zz_6_;
  assign _zz_5_ = _zz_4_[0 : 0];
  assign _zz_3_ = _zz_5_;
  assign io_pop_payload_mode = _zz_3_;
  assign io_pop_payload_args = _zz_4_[9 : 1];
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_11_};
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      logic_pushPtr_value <= (5'b00000);
      logic_popPtr_value <= (5'b00000);
      logic_risingOccupancy <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamFifo_3_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      input   io_flush,
      output [5:0] io_occupancy,
      output [5:0] io_availability,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [7:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [4:0] _zz_5_;
  wire [0:0] _zz_6_;
  wire [4:0] _zz_7_;
  wire [4:0] _zz_8_;
  wire  _zz_9_;
  reg  _zz_1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [4:0] logic_pushPtr_valueNext;
  reg [4:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [4:0] logic_popPtr_valueNext;
  reg [4:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_2_;
  wire [4:0] logic_ptrDif;
  reg [7:0] logic_ram [0:31];
  assign _zz_4_ = logic_pushPtr_willIncrement;
  assign _zz_5_ = {4'd0, _zz_4_};
  assign _zz_6_ = logic_popPtr_willIncrement;
  assign _zz_7_ = {4'd0, _zz_6_};
  assign _zz_8_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_9_ = 1'b1;
  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_1_) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_9_) begin
      _zz_3_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      _zz_1_ = 1'b1;
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (5'b11111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_5_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (5'b00000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (5'b11111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_7_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (5'b00000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2_ && (! logic_full))));
  assign io_pop_payload = _zz_3_;
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_8_};
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      logic_pushPtr_value <= (5'b00000);
      logic_popPtr_value <= (5'b00000);
      logic_risingOccupancy <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module I2cSlave (
      output  io_i2c_sda_write,
      input   io_i2c_sda_read,
      output  io_i2c_scl_write,
      input   io_i2c_scl_read,
      input  [9:0] io_config_samplingClockDivider,
      input  [19:0] io_config_timeout,
      input  [5:0] io_config_tsuData,
      output reg `I2cSlaveCmdMode_defaultEncoding_type io_bus_cmd_kind,
      output  io_bus_cmd_data,
      input   io_bus_rsp_valid,
      input   io_bus_rsp_enable,
      input   io_bus_rsp_data,
      output  io_internals_inFrame,
      output  io_internals_sdaRead,
      output  io_internals_sclRead,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_10_;
  wire  _zz_11_;
  wire  bufferCC_5__io_dataOut;
  wire  bufferCC_6__io_dataOut;
  wire  _zz_12_;
  wire  _zz_13_;
  reg [9:0] filter_timer_counter;
  wire  filter_timer_tick;
  wire  filter_sampler_sclSync;
  wire  filter_sampler_sdaSync;
  wire  filter_sampler_sclSamples_0;
  wire  filter_sampler_sclSamples_1;
  wire  filter_sampler_sclSamples_2;
  wire  _zz_1_;
  reg  _zz_2_;
  reg  _zz_3_;
  wire  filter_sampler_sdaSamples_0;
  wire  filter_sampler_sdaSamples_1;
  wire  filter_sampler_sdaSamples_2;
  wire  _zz_4_;
  reg  _zz_5_;
  reg  _zz_6_;
  reg  filter_sda;
  reg  filter_scl;
  wire  sclEdge_rise;
  wire  sclEdge_fall;
  wire  sclEdge_toggle;
  reg  filter_scl_regNext;
  wire  sdaEdge_rise;
  wire  sdaEdge_fall;
  wire  sdaEdge_toggle;
  reg  filter_sda_regNext;
  wire  detector_start;
  wire  detector_stop;
  reg [5:0] tsuData_counter;
  wire  tsuData_done;
  reg  tsuData_reset;
  reg  ctrl_inFrame;
  reg  ctrl_inFrameData;
  reg  ctrl_sdaWrite;
  reg  ctrl_sclWrite;
  wire  ctrl_rspBufferIn_valid;
  wire  ctrl_rspBufferIn_ready;
  wire  ctrl_rspBufferIn_payload_enable;
  wire  ctrl_rspBufferIn_payload_data;
  wire  ctrl_rspBufferIn_m2sPipe_valid;
  reg  ctrl_rspBufferIn_m2sPipe_ready;
  wire  ctrl_rspBufferIn_m2sPipe_payload_enable;
  wire  ctrl_rspBufferIn_m2sPipe_payload_data;
  reg  _zz_7_;
  reg  _zz_8_;
  reg  _zz_9_;
  wire  ctrl_rspAhead_valid;
  wire  ctrl_rspAhead_payload_enable;
  wire  ctrl_rspAhead_payload_data;
  reg [19:0] timeout_counter;
  reg  timeout_tick;
  reg  ctrl_sclWrite_regNext;
  reg  ctrl_sdaWrite_regNext;
  `ifndef SYNTHESIS
  reg [55:0] io_bus_cmd_kind_string;
  `endif

  assign _zz_12_ = (detector_stop || timeout_tick);
  assign _zz_13_ = (sclEdge_toggle || (! ctrl_inFrame));
  BufferCC bufferCC_5_ ( 
    .io_initial(_zz_10_),
    .io_dataIn(io_i2c_scl_read),
    .io_dataOut(bufferCC_5__io_dataOut),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  BufferCC bufferCC_6_ ( 
    .io_initial(_zz_11_),
    .io_dataIn(io_i2c_sda_read),
    .io_dataOut(bufferCC_6__io_dataOut),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_bus_cmd_kind)
      `I2cSlaveCmdMode_defaultEncoding_NONE : io_bus_cmd_kind_string = "NONE   ";
      `I2cSlaveCmdMode_defaultEncoding_START : io_bus_cmd_kind_string = "START  ";
      `I2cSlaveCmdMode_defaultEncoding_RESTART : io_bus_cmd_kind_string = "RESTART";
      `I2cSlaveCmdMode_defaultEncoding_STOP : io_bus_cmd_kind_string = "STOP   ";
      `I2cSlaveCmdMode_defaultEncoding_DROP : io_bus_cmd_kind_string = "DROP   ";
      `I2cSlaveCmdMode_defaultEncoding_DRIVE : io_bus_cmd_kind_string = "DRIVE  ";
      `I2cSlaveCmdMode_defaultEncoding_READ : io_bus_cmd_kind_string = "READ   ";
      default : io_bus_cmd_kind_string = "???????";
    endcase
  end
  `endif

  assign filter_timer_tick = (filter_timer_counter == (10'b0000000000));
  assign _zz_10_ = 1'b1;
  assign filter_sampler_sclSync = bufferCC_5__io_dataOut;
  assign _zz_11_ = 1'b1;
  assign filter_sampler_sdaSync = bufferCC_6__io_dataOut;
  assign _zz_1_ = filter_sampler_sclSync;
  assign filter_sampler_sclSamples_0 = _zz_1_;
  assign filter_sampler_sclSamples_1 = _zz_2_;
  assign filter_sampler_sclSamples_2 = _zz_3_;
  assign _zz_4_ = filter_sampler_sdaSync;
  assign filter_sampler_sdaSamples_0 = _zz_4_;
  assign filter_sampler_sdaSamples_1 = _zz_5_;
  assign filter_sampler_sdaSamples_2 = _zz_6_;
  assign sclEdge_rise = ((! filter_scl_regNext) && filter_scl);
  assign sclEdge_fall = (filter_scl_regNext && (! filter_scl));
  assign sclEdge_toggle = (filter_scl_regNext != filter_scl);
  assign sdaEdge_rise = ((! filter_sda_regNext) && filter_sda);
  assign sdaEdge_fall = (filter_sda_regNext && (! filter_sda));
  assign sdaEdge_toggle = (filter_sda_regNext != filter_sda);
  assign detector_start = (filter_scl && sdaEdge_fall);
  assign detector_stop = (filter_scl && sdaEdge_rise);
  assign tsuData_done = (tsuData_counter == (6'b000000));
  always @ (*) begin
    tsuData_reset = 1'b0;
    ctrl_sdaWrite = 1'b1;
    ctrl_sclWrite = 1'b1;
    ctrl_rspBufferIn_m2sPipe_ready = 1'b0;
    io_bus_cmd_kind = `I2cSlaveCmdMode_defaultEncoding_NONE;
    if(ctrl_inFrame)begin
      if(sclEdge_rise)begin
        io_bus_cmd_kind = `I2cSlaveCmdMode_defaultEncoding_READ;
      end
      if(sclEdge_fall)begin
        ctrl_rspBufferIn_m2sPipe_ready = 1'b1;
      end
    end
    if(ctrl_inFrameData)begin
      if(((! ctrl_rspBufferIn_m2sPipe_valid) || ctrl_rspBufferIn_m2sPipe_ready))begin
        io_bus_cmd_kind = `I2cSlaveCmdMode_defaultEncoding_DRIVE;
      end
      if(((! ctrl_rspAhead_valid) || (ctrl_rspAhead_payload_enable && (! tsuData_done))))begin
        ctrl_sclWrite = 1'b0;
      end
      tsuData_reset = (! ctrl_rspAhead_valid);
      if((ctrl_rspAhead_valid && ctrl_rspAhead_payload_enable))begin
        ctrl_sdaWrite = ctrl_rspAhead_payload_data;
      end
    end
    if(detector_start)begin
      io_bus_cmd_kind = (ctrl_inFrame ? `I2cSlaveCmdMode_defaultEncoding_RESTART : `I2cSlaveCmdMode_defaultEncoding_START);
    end
    if(_zz_12_)begin
      if(ctrl_inFrame)begin
        io_bus_cmd_kind = (timeout_tick ? `I2cSlaveCmdMode_defaultEncoding_DROP : `I2cSlaveCmdMode_defaultEncoding_STOP);
      end
    end
  end

  assign ctrl_rspBufferIn_ready = ((1'b1 && (! ctrl_rspBufferIn_m2sPipe_valid)) || ctrl_rspBufferIn_m2sPipe_ready);
  assign ctrl_rspBufferIn_m2sPipe_valid = _zz_7_;
  assign ctrl_rspBufferIn_m2sPipe_payload_enable = _zz_8_;
  assign ctrl_rspBufferIn_m2sPipe_payload_data = _zz_9_;
  assign ctrl_rspAhead_valid = (ctrl_rspBufferIn_m2sPipe_valid ? ctrl_rspBufferIn_m2sPipe_valid : ctrl_rspBufferIn_valid);
  assign ctrl_rspAhead_payload_enable = (ctrl_rspBufferIn_m2sPipe_valid ? ctrl_rspBufferIn_m2sPipe_payload_enable : ctrl_rspBufferIn_payload_enable);
  assign ctrl_rspAhead_payload_data = (ctrl_rspBufferIn_m2sPipe_valid ? ctrl_rspBufferIn_m2sPipe_payload_data : ctrl_rspBufferIn_payload_data);
  assign ctrl_rspBufferIn_valid = io_bus_rsp_valid;
  assign ctrl_rspBufferIn_payload_enable = io_bus_rsp_enable;
  assign ctrl_rspBufferIn_payload_data = io_bus_rsp_data;
  assign io_bus_cmd_data = filter_sda;
  always @ (*) begin
    timeout_tick = (timeout_counter == (20'b00000000000000000000));
    if(_zz_13_)begin
      timeout_tick = 1'b0;
    end
  end

  assign io_internals_inFrame = ctrl_inFrame;
  assign io_internals_sdaRead = filter_sda;
  assign io_internals_sclRead = filter_scl;
  assign io_i2c_scl_write = ctrl_sclWrite_regNext;
  assign io_i2c_sda_write = ctrl_sdaWrite_regNext;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      filter_timer_counter <= (10'b0000000000);
      _zz_2_ <= 1'b1;
      _zz_3_ <= 1'b1;
      _zz_5_ <= 1'b1;
      _zz_6_ <= 1'b1;
      filter_sda <= 1'b1;
      filter_scl <= 1'b1;
      filter_scl_regNext <= 1'b1;
      filter_sda_regNext <= 1'b1;
      tsuData_counter <= (6'b000000);
      ctrl_inFrame <= 1'b0;
      ctrl_inFrameData <= 1'b0;
      _zz_7_ <= 1'b0;
      timeout_counter <= (20'b00000000000000000000);
      ctrl_sclWrite_regNext <= 1'b1;
      ctrl_sdaWrite_regNext <= 1'b1;
    end else begin
      filter_timer_counter <= (filter_timer_counter - (10'b0000000001));
      if(filter_timer_tick)begin
        filter_timer_counter <= io_config_samplingClockDivider;
      end
      if(filter_timer_tick)begin
        _zz_2_ <= _zz_1_;
      end
      if(filter_timer_tick)begin
        _zz_3_ <= _zz_2_;
      end
      if(filter_timer_tick)begin
        _zz_5_ <= _zz_4_;
      end
      if(filter_timer_tick)begin
        _zz_6_ <= _zz_5_;
      end
      if(filter_timer_tick)begin
        if((((filter_sampler_sdaSamples_0 != filter_sda) && (filter_sampler_sdaSamples_1 != filter_sda)) && (filter_sampler_sdaSamples_2 != filter_sda)))begin
          filter_sda <= filter_sampler_sdaSamples_2;
        end
        if((((filter_sampler_sclSamples_0 != filter_scl) && (filter_sampler_sclSamples_1 != filter_scl)) && (filter_sampler_sclSamples_2 != filter_scl)))begin
          filter_scl <= filter_sampler_sclSamples_2;
        end
      end
      filter_scl_regNext <= filter_scl;
      filter_sda_regNext <= filter_sda;
      if((! tsuData_done))begin
        tsuData_counter <= (tsuData_counter - (6'b000001));
      end
      if(tsuData_reset)begin
        tsuData_counter <= io_config_tsuData;
      end
      if(ctrl_rspBufferIn_ready)begin
        _zz_7_ <= ctrl_rspBufferIn_valid;
      end
      if(ctrl_inFrame)begin
        if(sclEdge_fall)begin
          ctrl_inFrameData <= 1'b1;
        end
      end
      if(detector_start)begin
        ctrl_inFrame <= 1'b1;
        ctrl_inFrameData <= 1'b0;
      end
      timeout_counter <= (timeout_counter - (20'b00000000000000000001));
      if(_zz_13_)begin
        timeout_counter <= io_config_timeout;
      end
      if(_zz_12_)begin
        ctrl_inFrame <= 1'b0;
        ctrl_inFrameData <= 1'b0;
      end
      ctrl_sclWrite_regNext <= ctrl_sclWrite;
      ctrl_sdaWrite_regNext <= ctrl_sdaWrite;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(ctrl_rspBufferIn_ready)begin
      _zz_8_ <= ctrl_rspBufferIn_payload_enable;
      _zz_9_ <= ctrl_rspBufferIn_payload_data;
    end
  end

endmodule

module PulseInCtrl (
      input   io_pulseIn_pin,
      input  [31:0] io_timeout,
      input   io_value,
      input   io_req,
      output [31:0] io_pulseLength,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [31:0] _zz_1_;
  reg [31:0] pulseOut;
  reg [31:0] micros;
  reg [7:0] counter;
  reg  req;
  reg [1:0] state;
  assign _zz_1_ = (io_timeout - (32'b00000000000000000000000000000001));
  assign io_pulseLength = pulseOut;
  always @ (posedge toplevel_io_mainClk) begin
    if(io_req)begin
      req <= 1'b1;
      pulseOut <= (32'b00000000000000000000000000000000);
    end
    if(req)begin
      counter <= (counter + (8'b00000001));
      if((counter == (8'b00110001)))begin
        micros <= (micros + (32'b00000000000000000000000000000001));
        counter <= (8'b00000000);
      end
      if((((32'b00000000000000000000000000000000) < io_timeout) && (_zz_1_ <= micros)))begin
        req <= 1'b0;
        pulseOut <= (32'b11111111111111111111111111111111);
      end else begin
        if(((state == (2'b00)) && (io_pulseIn_pin != io_value)))begin
          state <= (2'b01);
        end else begin
          if(((state == (2'b01)) && (io_pulseIn_pin == io_value)))begin
            state <= (2'b10);
            counter <= (8'b00000000);
            micros <= (32'b00000000000000000000000000000000);
          end else begin
            if(((state == (2'b10)) && (io_pulseIn_pin != io_value)))begin
              if((micros == (32'b00000000000000000000000000000000)))begin
                pulseOut <= (32'b11111111111111111111111111111111);
              end else begin
                pulseOut <= micros;
              end
              req <= 1'b0;
            end
          end
        end
      end
    end else begin
      counter <= (8'b00000000);
      micros <= (32'b00000000000000000000000000000000);
      state <= (2'b00);
    end
  end

endmodule

module SevenSegmentCtrl (
      output  io_sevenSegment_digitPin,
      output [6:0] io_sevenSegment_segPins,
      input  [7:0] io_value,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [6:0] _zz_1_;
  wire [3:0] _zz_2_;
  wire [6:0] segROM_0;
  wire [6:0] segROM_1;
  wire [6:0] segROM_2;
  wire [6:0] segROM_3;
  wire [6:0] segROM_4;
  wire [6:0] segROM_5;
  wire [6:0] segROM_6;
  wire [6:0] segROM_7;
  wire [6:0] segROM_8;
  wire [6:0] segROM_9;
  wire [6:0] segROM_10;
  wire [6:0] segROM_11;
  wire [6:0] segROM_12;
  wire [6:0] segROM_13;
  wire [6:0] segROM_14;
  wire [6:0] segROM_15;
  reg [23:0] prescaler_1_;
  reg  digPos;
  reg [6:0] segOut;
  assign _zz_2_ = (digPos ? io_value[3 : 0] : io_value[7 : 4]);
  always @(*) begin
    case(_zz_2_)
      4'b0000 : begin
        _zz_1_ = segROM_0;
      end
      4'b0001 : begin
        _zz_1_ = segROM_1;
      end
      4'b0010 : begin
        _zz_1_ = segROM_2;
      end
      4'b0011 : begin
        _zz_1_ = segROM_3;
      end
      4'b0100 : begin
        _zz_1_ = segROM_4;
      end
      4'b0101 : begin
        _zz_1_ = segROM_5;
      end
      4'b0110 : begin
        _zz_1_ = segROM_6;
      end
      4'b0111 : begin
        _zz_1_ = segROM_7;
      end
      4'b1000 : begin
        _zz_1_ = segROM_8;
      end
      4'b1001 : begin
        _zz_1_ = segROM_9;
      end
      4'b1010 : begin
        _zz_1_ = segROM_10;
      end
      4'b1011 : begin
        _zz_1_ = segROM_11;
      end
      4'b1100 : begin
        _zz_1_ = segROM_12;
      end
      4'b1101 : begin
        _zz_1_ = segROM_13;
      end
      4'b1110 : begin
        _zz_1_ = segROM_14;
      end
      default : begin
        _zz_1_ = segROM_15;
      end
    endcase
  end

  assign segROM_0 = (7'b1111110);
  assign segROM_1 = (7'b0110000);
  assign segROM_2 = (7'b1101101);
  assign segROM_3 = (7'b1111001);
  assign segROM_4 = (7'b0110011);
  assign segROM_5 = (7'b1011011);
  assign segROM_6 = (7'b1011111);
  assign segROM_7 = (7'b1110000);
  assign segROM_8 = (7'b1111111);
  assign segROM_9 = (7'b1111011);
  assign segROM_10 = (7'b1110111);
  assign segROM_11 = (7'b0011111);
  assign segROM_12 = (7'b1101110);
  assign segROM_13 = (7'b0111101);
  assign segROM_14 = (7'b1001111);
  assign segROM_15 = (7'b1000111);
  assign io_sevenSegment_digitPin = digPos;
  assign io_sevenSegment_segPins = segOut;
  always @ (posedge toplevel_io_mainClk) begin
    prescaler_1_ <= (prescaler_1_ + (24'b000000000000000000000001));
    if((prescaler_1_ == (24'b000000001100001101010000)))begin
      prescaler_1_ <= (24'b000000000000000000000000);
      digPos <= (! digPos);
      segOut <= _zz_1_;
    end
  end

endmodule

module ShiftInCtrl (
      input   io_shiftIn_dataPin,
      output  io_shiftIn_clockPin,
      output [7:0] io_value,
      input   io_req,
      input  [11:0] io_preScale,
      input   io_bitOrder,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [11:0] _zz_1_;
  reg [7:0] shiftReg;
  reg [3:0] bitCounter;
  reg  clockReg;
  reg [11:0] prescaler_1_;
  assign _zz_1_ = (io_preScale - (12'b000000000001));
  assign io_value = shiftReg;
  assign io_shiftIn_clockPin = clockReg;
  always @ (posedge toplevel_io_mainClk) begin
    if(io_req)begin
      bitCounter <= (4'b1000);
      prescaler_1_ <= (12'b000000000000);
      shiftReg <= (8'b00000000);
    end
    if(((4'b0000) < bitCounter))begin
      prescaler_1_ <= (prescaler_1_ + (12'b000000000001));
      if((prescaler_1_ == _zz_1_))begin
        prescaler_1_ <= (12'b000000000000);
        clockReg <= (! clockReg);
        if(clockReg)begin
          if(io_bitOrder)begin
            shiftReg <= (shiftReg <<< 1);
          end else begin
            shiftReg <= (shiftReg >>> 1);
          end
        end else begin
          bitCounter <= (bitCounter - (4'b0001));
          if(io_bitOrder)begin
            shiftReg[0] <= io_shiftIn_dataPin;
          end else begin
            shiftReg[7] <= io_shiftIn_dataPin;
          end
        end
      end
    end else begin
      clockReg <= 1'b1;
    end
  end

endmodule

module QspiCtrl (
      input   io_qspi_qck,
      input   io_qspi_qss,
      input  [3:0] io_qspi_qd_read,
      output [3:0] io_qspi_qd_write,
      output [3:0] io_qspi_qd_writeEnable,
      output [9:0] io_a0,
      output [9:0] io_a1,
      output [9:0] io_a2,
      output [9:0] io_a3,
      output [9:0] io_a4,
      output [9:0] io_a5,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [3:0] _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  rx_io_rxReady;
  wire [7:0] rx_io_rxData;
  wire [3:0] rx_io_byteNumber;
  wire [3:0] tx_io_qd;
  wire  tx_io_txReady;
  reg [2:0] qssR;
  reg [7:0] qdR;
  reg [2:0] qckR;
  wire  qckRise;
  wire  qckFall;
  wire  qssRise;
  reg [3:0] readEnable;
  reg [7:0] rxData;
  reg [9:0] a0;
  reg [9:0] a1;
  reg [9:0] a2;
  reg [9:0] a3;
  reg [9:0] a4;
  reg [9:0] a5;
  QspiSlaveRX rx ( 
    .io_qckRise(qckRise),
    .io_qssRise(qssRise),
    .io_qd(_zz_1_),
    .io_rxReady(rx_io_rxReady),
    .io_rxData(rx_io_rxData),
    .io_byteNumber(rx_io_byteNumber),
    .io_readEnable(_zz_2_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  QspiSlaveTX tx ( 
    .io_qckFall(qckFall),
    .io_qssRise(qssRise),
    .io_qd(tx_io_qd),
    .io_txReady(tx_io_txReady),
    .io_txData(rxData),
    .io_writeEnable(_zz_3_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign qckRise = (qckR[2 : 1] == (2'b01));
  assign qckFall = (qckR[2 : 1] == (2'b10));
  assign qssRise = (qssR[2 : 1] == (2'b01));
  assign _zz_1_ = qdR[7 : 4];
  assign io_qspi_qd_write = tx_io_qd;
  assign io_qspi_qd_writeEnable = (~ readEnable);
  assign _zz_2_ = (readEnable != (4'b0000));
  assign _zz_3_ = (readEnable == (4'b0000));
  assign io_a0 = a0;
  assign io_a1 = a1;
  assign io_a2 = a2;
  assign io_a3 = a3;
  assign io_a4 = a4;
  assign io_a5 = a5;
  always @ (posedge toplevel_io_mainClk) begin
    qssR <= {qssR[1 : 0],io_qspi_qss};
    qdR <= {qdR[3 : 0],io_qspi_qd_read};
    qckR <= {qckR[1 : 0],io_qspi_qck};
    if(qssRise)begin
      readEnable <= (~ readEnable);
    end
    if(rx_io_rxReady)begin
      rxData <= rx_io_rxData;
    end
    if(rx_io_rxReady)begin
      if((rx_io_byteNumber == (4'b0010)))begin
        a0[7 : 0] <= rx_io_rxData;
      end
      if((rx_io_byteNumber == (4'b0011)))begin
        a0[9 : 8] <= rx_io_rxData[1 : 0];
      end
      if((rx_io_byteNumber == (4'b0100)))begin
        a1[7 : 0] <= rx_io_rxData;
      end
      if((rx_io_byteNumber == (4'b0101)))begin
        a1[9 : 8] <= rx_io_rxData[1 : 0];
      end
      if((rx_io_byteNumber == (4'b0110)))begin
        a2[7 : 0] <= rx_io_rxData;
      end
      if((rx_io_byteNumber == (4'b0111)))begin
        a2[9 : 8] <= rx_io_rxData[1 : 0];
      end
      if((rx_io_byteNumber == (4'b1000)))begin
        a3[7 : 0] <= rx_io_rxData;
      end
      if((rx_io_byteNumber == (4'b1001)))begin
        a3[9 : 8] <= rx_io_rxData[1 : 0];
      end
      if((rx_io_byteNumber == (4'b1010)))begin
        a4[7 : 0] <= rx_io_rxData;
      end
      if((rx_io_byteNumber == (4'b1011)))begin
        a4[9 : 8] <= rx_io_rxData[1 : 0];
      end
      if((rx_io_byteNumber == (4'b1100)))begin
        a5[7 : 0] <= rx_io_rxData;
      end
      if((rx_io_byteNumber == (4'b1101)))begin
        a5[9 : 8] <= rx_io_rxData[1 : 0];
      end
    end
  end

endmodule

module BufferCC_4_ (
      input   io_dataIn,
      output  io_dataOut,
      input   toplevel_io_mainClk);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_io_mainClk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module MuraxMasterArbiter (
      input   io_iBus_cmd_valid,
      output reg  io_iBus_cmd_ready,
      input  [31:0] io_iBus_cmd_payload_pc,
      output  io_iBus_rsp_valid,
      output  io_iBus_rsp_payload_error,
      output [31:0] io_iBus_rsp_payload_inst,
      input   io_dBus_cmd_valid,
      output reg  io_dBus_cmd_ready,
      input   io_dBus_cmd_payload_wr,
      input  [31:0] io_dBus_cmd_payload_address,
      input  [31:0] io_dBus_cmd_payload_data,
      input  [1:0] io_dBus_cmd_payload_size,
      output  io_dBus_rsp_ready,
      output  io_dBus_rsp_error,
      output [31:0] io_dBus_rsp_data,
      output reg  io_masterBus_cmd_valid,
      input   io_masterBus_cmd_ready,
      output  io_masterBus_cmd_payload_write,
      output [31:0] io_masterBus_cmd_payload_address,
      output [31:0] io_masterBus_cmd_payload_data,
      output [3:0] io_masterBus_cmd_payload_mask,
      input   io_masterBus_rsp_valid,
      input  [31:0] io_masterBus_rsp_payload_data,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [3:0] _zz_1_;
  reg  rspPending;
  reg  rspTarget;
  always @ (*) begin
    io_masterBus_cmd_valid = (io_iBus_cmd_valid || io_dBus_cmd_valid);
    io_iBus_cmd_ready = (io_masterBus_cmd_ready && (! io_dBus_cmd_valid));
    io_dBus_cmd_ready = io_masterBus_cmd_ready;
    if((rspPending && (! io_masterBus_rsp_valid)))begin
      io_iBus_cmd_ready = 1'b0;
      io_dBus_cmd_ready = 1'b0;
      io_masterBus_cmd_valid = 1'b0;
    end
  end

  assign io_masterBus_cmd_payload_write = (io_dBus_cmd_valid && io_dBus_cmd_payload_wr);
  assign io_masterBus_cmd_payload_address = (io_dBus_cmd_valid ? io_dBus_cmd_payload_address : io_iBus_cmd_payload_pc);
  assign io_masterBus_cmd_payload_data = io_dBus_cmd_payload_data;
  always @ (*) begin
    case(io_dBus_cmd_payload_size)
      2'b00 : begin
        _zz_1_ = (4'b0001);
      end
      2'b01 : begin
        _zz_1_ = (4'b0011);
      end
      default : begin
        _zz_1_ = (4'b1111);
      end
    endcase
  end

  assign io_masterBus_cmd_payload_mask = (_zz_1_ <<< io_dBus_cmd_payload_address[1 : 0]);
  assign io_iBus_rsp_valid = (io_masterBus_rsp_valid && (! rspTarget));
  assign io_iBus_rsp_payload_inst = io_masterBus_rsp_payload_data;
  assign io_iBus_rsp_payload_error = 1'b0;
  assign io_dBus_rsp_ready = (io_masterBus_rsp_valid && rspTarget);
  assign io_dBus_rsp_data = io_masterBus_rsp_payload_data;
  assign io_dBus_rsp_error = 1'b0;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      rspPending <= 1'b0;
      rspTarget <= 1'b0;
    end else begin
      if(io_masterBus_rsp_valid)begin
        rspPending <= 1'b0;
      end
      if(((io_masterBus_cmd_valid && io_masterBus_cmd_ready) && (! io_masterBus_cmd_payload_write)))begin
        rspTarget <= io_dBus_cmd_valid;
        rspPending <= 1'b1;
      end
    end
  end

endmodule

module VexRiscv (
      output  iBus_cmd_valid,
      input   iBus_cmd_ready,
      output [31:0] iBus_cmd_payload_pc,
      input   iBus_rsp_valid,
      input   iBus_rsp_payload_error,
      input  [31:0] iBus_rsp_payload_inst,
      input   timerInterrupt,
      input   externalInterrupt,
      input   debug_bus_cmd_valid,
      output reg  debug_bus_cmd_ready,
      input   debug_bus_cmd_payload_wr,
      input  [7:0] debug_bus_cmd_payload_address,
      input  [31:0] debug_bus_cmd_payload_data,
      output reg [31:0] debug_bus_rsp_data,
      output  debug_resetOut,
      output  dBus_cmd_valid,
      input   dBus_cmd_ready,
      output  dBus_cmd_payload_wr,
      output [31:0] dBus_cmd_payload_address,
      output [31:0] dBus_cmd_payload_data,
      output [1:0] dBus_cmd_payload_size,
      input   dBus_rsp_ready,
      input   dBus_rsp_error,
      input  [31:0] dBus_rsp_data,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset,
      input   toplevel_resetCtrl_mainClkReset);
  wire  _zz_151_;
  reg [31:0] _zz_152_;
  reg [31:0] _zz_153_;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  wire [0:0] IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy;
  wire  _zz_154_;
  wire  _zz_155_;
  wire  _zz_156_;
  wire  _zz_157_;
  wire  _zz_158_;
  wire  _zz_159_;
  wire  _zz_160_;
  wire  _zz_161_;
  wire [5:0] _zz_162_;
  wire [1:0] _zz_163_;
  wire [1:0] _zz_164_;
  wire  _zz_165_;
  wire [1:0] _zz_166_;
  wire [1:0] _zz_167_;
  wire [2:0] _zz_168_;
  wire [31:0] _zz_169_;
  wire [2:0] _zz_170_;
  wire [0:0] _zz_171_;
  wire [2:0] _zz_172_;
  wire [0:0] _zz_173_;
  wire [2:0] _zz_174_;
  wire [0:0] _zz_175_;
  wire [2:0] _zz_176_;
  wire [0:0] _zz_177_;
  wire [0:0] _zz_178_;
  wire [0:0] _zz_179_;
  wire [0:0] _zz_180_;
  wire [0:0] _zz_181_;
  wire [0:0] _zz_182_;
  wire [0:0] _zz_183_;
  wire [0:0] _zz_184_;
  wire [0:0] _zz_185_;
  wire [0:0] _zz_186_;
  wire [0:0] _zz_187_;
  wire [2:0] _zz_188_;
  wire [4:0] _zz_189_;
  wire [11:0] _zz_190_;
  wire [11:0] _zz_191_;
  wire [31:0] _zz_192_;
  wire [31:0] _zz_193_;
  wire [31:0] _zz_194_;
  wire [31:0] _zz_195_;
  wire [1:0] _zz_196_;
  wire [31:0] _zz_197_;
  wire [1:0] _zz_198_;
  wire [1:0] _zz_199_;
  wire [31:0] _zz_200_;
  wire [32:0] _zz_201_;
  wire [19:0] _zz_202_;
  wire [11:0] _zz_203_;
  wire [11:0] _zz_204_;
  wire [0:0] _zz_205_;
  wire [0:0] _zz_206_;
  wire [0:0] _zz_207_;
  wire [0:0] _zz_208_;
  wire [0:0] _zz_209_;
  wire [0:0] _zz_210_;
  wire  _zz_211_;
  wire  _zz_212_;
  wire [31:0] _zz_213_;
  wire  _zz_214_;
  wire  _zz_215_;
  wire [0:0] _zz_216_;
  wire [0:0] _zz_217_;
  wire [2:0] _zz_218_;
  wire [2:0] _zz_219_;
  wire  _zz_220_;
  wire [0:0] _zz_221_;
  wire [17:0] _zz_222_;
  wire [31:0] _zz_223_;
  wire  _zz_224_;
  wire  _zz_225_;
  wire [31:0] _zz_226_;
  wire [31:0] _zz_227_;
  wire [0:0] _zz_228_;
  wire [0:0] _zz_229_;
  wire [4:0] _zz_230_;
  wire [4:0] _zz_231_;
  wire  _zz_232_;
  wire [0:0] _zz_233_;
  wire [14:0] _zz_234_;
  wire [31:0] _zz_235_;
  wire [31:0] _zz_236_;
  wire [0:0] _zz_237_;
  wire [1:0] _zz_238_;
  wire  _zz_239_;
  wire [0:0] _zz_240_;
  wire [0:0] _zz_241_;
  wire  _zz_242_;
  wire [0:0] _zz_243_;
  wire [11:0] _zz_244_;
  wire [31:0] _zz_245_;
  wire [31:0] _zz_246_;
  wire [31:0] _zz_247_;
  wire [31:0] _zz_248_;
  wire [31:0] _zz_249_;
  wire [0:0] _zz_250_;
  wire [1:0] _zz_251_;
  wire [2:0] _zz_252_;
  wire [2:0] _zz_253_;
  wire  _zz_254_;
  wire [0:0] _zz_255_;
  wire [8:0] _zz_256_;
  wire [31:0] _zz_257_;
  wire [31:0] _zz_258_;
  wire [31:0] _zz_259_;
  wire [31:0] _zz_260_;
  wire [31:0] _zz_261_;
  wire [31:0] _zz_262_;
  wire [31:0] _zz_263_;
  wire  _zz_264_;
  wire  _zz_265_;
  wire  _zz_266_;
  wire [1:0] _zz_267_;
  wire [1:0] _zz_268_;
  wire  _zz_269_;
  wire [0:0] _zz_270_;
  wire [5:0] _zz_271_;
  wire [31:0] _zz_272_;
  wire [31:0] _zz_273_;
  wire [31:0] _zz_274_;
  wire [31:0] _zz_275_;
  wire [0:0] _zz_276_;
  wire [0:0] _zz_277_;
  wire [1:0] _zz_278_;
  wire [1:0] _zz_279_;
  wire  _zz_280_;
  wire [0:0] _zz_281_;
  wire [2:0] _zz_282_;
  wire [31:0] _zz_283_;
  wire  _zz_284_;
  wire [0:0] _zz_285_;
  wire [1:0] _zz_286_;
  wire [0:0] _zz_287_;
  wire [0:0] _zz_288_;
  wire [1:0] _zz_289_;
  wire [1:0] _zz_290_;
  wire [0:0] _zz_291_;
  wire [0:0] _zz_292_;
  wire [31:0] _zz_293_;
  wire [31:0] _zz_294_;
  wire [31:0] _zz_295_;
  wire [31:0] _zz_296_;
  wire [31:0] _zz_297_;
  wire [31:0] _zz_298_;
  wire [31:0] _zz_299_;
  wire [31:0] _zz_300_;
  wire [31:0] _zz_301_;
  wire [31:0] _zz_302_;
  wire `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_1_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_2_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_3_;
  wire [31:0] execute_BRANCH_CALC;
  wire [31:0] decode_RS1;
  wire  decode_MEMORY_ENABLE;
  wire [31:0] memory_MEMORY_READ_DATA;
  wire  decode_SRC_USE_SUB_LESS;
  wire [31:0] memory_PC;
  wire [31:0] writeBack_REGFILE_WRITE_DATA;
  wire [31:0] execute_REGFILE_WRITE_DATA;
  wire `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_4_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_5_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_6_;
  wire [31:0] decode_SRC1;
  wire [31:0] decode_SRC2;
  wire  decode_IS_CSR;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_7_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_8_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_9_;
  wire  decode_SRC_LESS_UNSIGNED;
  wire [31:0] decode_RS2;
  wire [31:0] writeBack_FORMAL_PC_NEXT;
  wire [31:0] memory_FORMAL_PC_NEXT;
  wire [31:0] execute_FORMAL_PC_NEXT;
  wire [31:0] decode_FORMAL_PC_NEXT;
  wire  decode_CSR_READ_OPCODE;
  wire  execute_BRANCH_DO;
  wire  decode_DO_EBREAK;
  wire  execute_BYPASSABLE_MEMORY_STAGE;
  wire  decode_BYPASSABLE_MEMORY_STAGE;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_10_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_11_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_12_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_13_;
  wire `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_14_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_15_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_16_;
  wire [1:0] memory_MEMORY_ADDRESS_LOW;
  wire [1:0] execute_MEMORY_ADDRESS_LOW;
  wire  decode_CSR_WRITE_OPCODE;
  wire  decode_BYPASSABLE_EXECUTE_STAGE;
  wire `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_17_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_18_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_19_;
  wire  execute_DO_EBREAK;
  wire  decode_IS_EBREAK;
  wire  _zz_20_;
  wire [31:0] memory_BRANCH_CALC;
  wire  memory_BRANCH_DO;
  wire [31:0] _zz_21_;
  wire [31:0] execute_PC;
  wire [31:0] execute_RS1;
  wire `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_22_;
  wire  _zz_23_;
  wire  decode_RS2_USE;
  wire  decode_RS1_USE;
  wire  execute_REGFILE_WRITE_VALID;
  wire  execute_BYPASSABLE_EXECUTE_STAGE;
  wire  memory_REGFILE_WRITE_VALID;
  wire  memory_BYPASSABLE_MEMORY_STAGE;
  wire  writeBack_REGFILE_WRITE_VALID;
  wire [31:0] memory_REGFILE_WRITE_DATA;
  wire `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_24_;
  wire  _zz_25_;
  wire [31:0] _zz_26_;
  wire [31:0] _zz_27_;
  wire  execute_SRC_LESS_UNSIGNED;
  wire  execute_SRC_USE_SUB_LESS;
  wire [31:0] _zz_28_;
  wire [31:0] _zz_29_;
  wire `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_30_;
  wire [31:0] _zz_31_;
  wire [31:0] _zz_32_;
  wire `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_33_;
  wire [31:0] _zz_34_;
  wire [31:0] execute_SRC_ADD_SUB;
  wire  execute_SRC_LESS;
  wire `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_35_;
  wire [31:0] _zz_36_;
  wire [31:0] execute_SRC2;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_37_;
  wire [31:0] _zz_38_;
  wire  _zz_39_;
  reg  _zz_40_;
  wire [31:0] _zz_41_;
  wire [31:0] _zz_42_;
  wire [31:0] decode_INSTRUCTION_ANTICIPATED;
  reg  decode_REGFILE_WRITE_VALID;
  wire  _zz_43_;
  wire  _zz_44_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_45_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_46_;
  wire  _zz_47_;
  wire  _zz_48_;
  wire  _zz_49_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_50_;
  wire  _zz_51_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_52_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_53_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_54_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_55_;
  wire  _zz_56_;
  wire  _zz_57_;
  wire  _zz_58_;
  wire  _zz_59_;
  reg [31:0] _zz_60_;
  wire [31:0] execute_SRC1;
  wire  execute_CSR_READ_OPCODE;
  wire  execute_CSR_WRITE_OPCODE;
  wire  execute_IS_CSR;
  wire `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_61_;
  wire `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_62_;
  wire  _zz_63_;
  wire  _zz_64_;
  wire `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_65_;
  reg [31:0] _zz_66_;
  wire  writeBack_MEMORY_ENABLE;
  wire [1:0] writeBack_MEMORY_ADDRESS_LOW;
  wire [31:0] writeBack_MEMORY_READ_DATA;
  wire [31:0] memory_INSTRUCTION;
  wire  memory_MEMORY_ENABLE;
  wire [31:0] _zz_67_;
  wire [1:0] _zz_68_;
  wire [31:0] execute_RS2;
  wire [31:0] execute_SRC_ADD;
  wire [31:0] execute_INSTRUCTION;
  wire  execute_ALIGNEMENT_FAULT;
  wire  execute_MEMORY_ENABLE;
  reg [31:0] _zz_69_;
  wire [31:0] _zz_70_;
  wire [31:0] _zz_71_;
  wire [31:0] _zz_72_;
  wire [31:0] _zz_73_;
  wire [31:0] writeBack_PC /* verilator public */ ;
  wire [31:0] writeBack_INSTRUCTION /* verilator public */ ;
  wire [31:0] decode_PC /* verilator public */ ;
  wire [31:0] decode_INSTRUCTION /* verilator public */ ;
  reg  decode_arbitration_haltItself /* verilator public */ ;
  reg  decode_arbitration_haltByOther;
  reg  decode_arbitration_removeIt;
  wire  decode_arbitration_flushAll /* verilator public */ ;
  wire  decode_arbitration_redoIt;
  reg  decode_arbitration_isValid /* verilator public */ ;
  wire  decode_arbitration_isStuck;
  wire  decode_arbitration_isStuckByOthers;
  wire  decode_arbitration_isFlushed;
  wire  decode_arbitration_isMoving;
  wire  decode_arbitration_isFiring;
  reg  execute_arbitration_haltItself;
  reg  execute_arbitration_haltByOther;
  reg  execute_arbitration_removeIt;
  reg  execute_arbitration_flushAll;
  wire  execute_arbitration_redoIt;
  reg  execute_arbitration_isValid;
  wire  execute_arbitration_isStuck;
  wire  execute_arbitration_isStuckByOthers;
  wire  execute_arbitration_isFlushed;
  wire  execute_arbitration_isMoving;
  wire  execute_arbitration_isFiring;
  reg  memory_arbitration_haltItself;
  wire  memory_arbitration_haltByOther;
  reg  memory_arbitration_removeIt;
  reg  memory_arbitration_flushAll;
  wire  memory_arbitration_redoIt;
  reg  memory_arbitration_isValid;
  wire  memory_arbitration_isStuck;
  wire  memory_arbitration_isStuckByOthers;
  wire  memory_arbitration_isFlushed;
  wire  memory_arbitration_isMoving;
  wire  memory_arbitration_isFiring;
  wire  writeBack_arbitration_haltItself;
  wire  writeBack_arbitration_haltByOther;
  reg  writeBack_arbitration_removeIt;
  wire  writeBack_arbitration_flushAll;
  wire  writeBack_arbitration_redoIt;
  reg  writeBack_arbitration_isValid /* verilator public */ ;
  wire  writeBack_arbitration_isStuck;
  wire  writeBack_arbitration_isStuckByOthers;
  wire  writeBack_arbitration_isFlushed;
  wire  writeBack_arbitration_isMoving;
  wire  writeBack_arbitration_isFiring /* verilator public */ ;
  reg  _zz_74_;
  reg  _zz_75_;
  reg  _zz_76_;
  reg  _zz_77_;
  reg [31:0] _zz_78_;
  wire  contextSwitching;
  reg [1:0] CsrPlugin_privilege;
  reg  _zz_79_;
  wire  _zz_80_;
  wire [31:0] _zz_81_;
  reg  _zz_82_;
  reg  _zz_83_;
  wire  IBusSimplePlugin_jump_pcLoad_valid;
  wire [31:0] IBusSimplePlugin_jump_pcLoad_payload;
  wire [1:0] _zz_84_;
  wire  IBusSimplePlugin_fetchPc_preOutput_valid;
  wire  IBusSimplePlugin_fetchPc_preOutput_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_preOutput_payload;
  wire  _zz_85_;
  wire  IBusSimplePlugin_fetchPc_output_valid;
  wire  IBusSimplePlugin_fetchPc_output_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_output_payload;
  reg [31:0] IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg  IBusSimplePlugin_fetchPc_inc;
  reg  IBusSimplePlugin_fetchPc_propagatePc;
  reg [31:0] IBusSimplePlugin_fetchPc_pc;
  reg  IBusSimplePlugin_fetchPc_samplePcNext;
  reg  _zz_86_;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_0_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  reg  IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_1_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_2_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_2_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_2_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_2_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_2_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_2_inputSample;
  wire  _zz_87_;
  wire  _zz_88_;
  wire  _zz_89_;
  wire  _zz_90_;
  wire  _zz_91_;
  reg  _zz_92_;
  wire  _zz_93_;
  reg  _zz_94_;
  reg [31:0] _zz_95_;
  reg  IBusSimplePlugin_iBusRsp_readyForError;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
  wire  IBusSimplePlugin_injector_decodeInput_valid;
  wire  IBusSimplePlugin_injector_decodeInput_ready;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire  IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire  IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg  _zz_96_;
  reg [31:0] _zz_97_;
  reg  _zz_98_;
  reg [31:0] _zz_99_;
  reg  _zz_100_;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg  IBusSimplePlugin_injector_nextPcCalc_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_1;
  reg  IBusSimplePlugin_injector_nextPcCalc_2;
  reg  IBusSimplePlugin_injector_nextPcCalc_3;
  reg  IBusSimplePlugin_injector_decodeRemoved;
  reg [31:0] IBusSimplePlugin_injector_formal_rawInDecode;
  wire  IBusSimplePlugin_cmd_valid;
  wire  IBusSimplePlugin_cmd_ready;
  wire [31:0] IBusSimplePlugin_cmd_payload_pc;
  reg [2:0] IBusSimplePlugin_pendingCmd;
  wire [2:0] IBusSimplePlugin_pendingCmdNext;
  reg [2:0] IBusSimplePlugin_rspJoin_discardCounter;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_valid;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_ready;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  wire  iBus_rsp_takeWhen_valid;
  wire  iBus_rsp_takeWhen_payload_error;
  wire [31:0] iBus_rsp_takeWhen_payload_inst;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_pc;
  reg  IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  wire  IBusSimplePlugin_rspJoin_issueDetected;
  wire  IBusSimplePlugin_rspJoin_join_valid;
  wire  IBusSimplePlugin_rspJoin_join_ready;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_pc;
  wire  IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_join_payload_isRvc;
  wire  _zz_101_;
  wire  execute_DBusSimplePlugin_cmdSent;
  reg [31:0] _zz_102_;
  reg [3:0] _zz_103_;
  wire [3:0] execute_DBusSimplePlugin_formalMask;
  reg [31:0] writeBack_DBusSimplePlugin_rspShifted;
  wire  _zz_104_;
  reg [31:0] _zz_105_;
  wire  _zz_106_;
  reg [31:0] _zz_107_;
  reg [31:0] writeBack_DBusSimplePlugin_rspFormated;
  wire [1:0] CsrPlugin_misa_base;
  wire [25:0] CsrPlugin_misa_extensions;
  wire [1:0] CsrPlugin_mtvec_mode;
  wire [29:0] CsrPlugin_mtvec_base;
  reg [31:0] CsrPlugin_mepc;
  reg  CsrPlugin_mstatus_MIE;
  reg  CsrPlugin_mstatus_MPIE;
  reg [1:0] CsrPlugin_mstatus_MPP;
  reg  CsrPlugin_mip_MEIP;
  reg  CsrPlugin_mip_MTIP;
  reg  CsrPlugin_mip_MSIP;
  reg  CsrPlugin_mie_MEIE;
  reg  CsrPlugin_mie_MTIE;
  reg  CsrPlugin_mie_MSIE;
  reg  CsrPlugin_mcause_interrupt;
  reg [3:0] CsrPlugin_mcause_exceptionCode;
  reg [31:0] CsrPlugin_mtval;
  reg [63:0] CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg [63:0] CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  wire [31:0] CsrPlugin_medeleg;
  wire [31:0] CsrPlugin_mideleg;
  wire  _zz_108_;
  wire  _zz_109_;
  wire  _zz_110_;
  reg  CsrPlugin_interrupt;
  reg [3:0] CsrPlugin_interruptCode /* verilator public */ ;
  wire [1:0] CsrPlugin_interruptTargetPrivilege;
  wire  CsrPlugin_exception;
  wire  CsrPlugin_lastStageWasWfi;
  reg  CsrPlugin_pipelineLiberator_done;
  wire  CsrPlugin_interruptJump /* verilator public */ ;
  reg  CsrPlugin_hadException;
  wire [1:0] CsrPlugin_targetPrivilege;
  wire [3:0] CsrPlugin_trapCause;
  wire  execute_CsrPlugin_blockedBySideEffects;
  reg  execute_CsrPlugin_illegalAccess;
  reg  execute_CsrPlugin_illegalInstruction;
  reg [31:0] execute_CsrPlugin_readData;
  wire  execute_CsrPlugin_writeInstruction;
  wire  execute_CsrPlugin_readInstruction;
  wire  execute_CsrPlugin_writeEnable;
  wire  execute_CsrPlugin_readEnable;
  reg [31:0] execute_CsrPlugin_writeData;
  wire [11:0] execute_CsrPlugin_csrAddress;
  wire [23:0] _zz_111_;
  wire  _zz_112_;
  wire  _zz_113_;
  wire  _zz_114_;
  wire  _zz_115_;
  wire  _zz_116_;
  wire  _zz_117_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_118_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_119_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_120_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_121_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_122_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_123_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_124_;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress1;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress2;
  wire [31:0] decode_RegFilePlugin_rs1Data;
  wire [31:0] decode_RegFilePlugin_rs2Data;
  reg  writeBack_RegFilePlugin_regFileWrite_valid /* verilator public */ ;
  wire [4:0] writeBack_RegFilePlugin_regFileWrite_payload_address /* verilator public */ ;
  wire [31:0] writeBack_RegFilePlugin_regFileWrite_payload_data /* verilator public */ ;
  reg  _zz_125_;
  reg [31:0] execute_IntAluPlugin_bitwise;
  reg [31:0] _zz_126_;
  reg [31:0] _zz_127_;
  wire  _zz_128_;
  reg [19:0] _zz_129_;
  wire  _zz_130_;
  reg [19:0] _zz_131_;
  reg [31:0] _zz_132_;
  wire [31:0] execute_SrcPlugin_addSub;
  wire  execute_SrcPlugin_less;
  reg  execute_LightShifterPlugin_isActive;
  wire  execute_LightShifterPlugin_isShift;
  reg [4:0] execute_LightShifterPlugin_amplitudeReg;
  wire [4:0] execute_LightShifterPlugin_amplitude;
  wire [31:0] execute_LightShifterPlugin_shiftInput;
  wire  execute_LightShifterPlugin_done;
  reg [31:0] _zz_133_;
  reg  _zz_134_;
  reg  _zz_135_;
  wire  _zz_136_;
  reg  _zz_137_;
  reg [4:0] _zz_138_;
  wire  execute_BranchPlugin_eq;
  wire [2:0] _zz_139_;
  reg  _zz_140_;
  reg  _zz_141_;
  wire [31:0] execute_BranchPlugin_branch_src1;
  wire  _zz_142_;
  reg [10:0] _zz_143_;
  wire  _zz_144_;
  reg [19:0] _zz_145_;
  wire  _zz_146_;
  reg [18:0] _zz_147_;
  reg [31:0] _zz_148_;
  wire [31:0] execute_BranchPlugin_branch_src2;
  wire [31:0] execute_BranchPlugin_branchAdder;
  reg  DebugPlugin_firstCycle;
  reg  DebugPlugin_secondCycle;
  reg  DebugPlugin_resetIt;
  reg  DebugPlugin_haltIt;
  reg  DebugPlugin_stepIt;
  reg  DebugPlugin_isPipActive;
  reg  DebugPlugin_isPipActive_regNext;
  wire  DebugPlugin_isPipBusy;
  reg  DebugPlugin_haltedByBreak;
  reg [31:0] DebugPlugin_busReadDataReg;
  reg  _zz_149_;
  reg  DebugPlugin_resetIt_regNext;
  reg `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg  decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg  decode_to_execute_CSR_WRITE_OPCODE;
  reg [1:0] execute_to_memory_MEMORY_ADDRESS_LOW;
  reg [1:0] memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg  decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg  execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg  decode_to_execute_REGFILE_WRITE_VALID;
  reg  execute_to_memory_REGFILE_WRITE_VALID;
  reg  memory_to_writeBack_REGFILE_WRITE_VALID;
  reg  decode_to_execute_DO_EBREAK;
  reg  execute_to_memory_BRANCH_DO;
  reg  decode_to_execute_CSR_READ_OPCODE;
  reg [31:0] decode_to_execute_FORMAL_PC_NEXT;
  reg [31:0] execute_to_memory_FORMAL_PC_NEXT;
  reg [31:0] memory_to_writeBack_FORMAL_PC_NEXT;
  reg [31:0] decode_to_execute_RS2;
  reg  decode_to_execute_SRC_LESS_UNSIGNED;
  reg `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg  decode_to_execute_IS_CSR;
  reg [31:0] decode_to_execute_SRC2;
  reg [31:0] decode_to_execute_SRC1;
  reg [31:0] decode_to_execute_INSTRUCTION;
  reg [31:0] execute_to_memory_INSTRUCTION;
  reg [31:0] memory_to_writeBack_INSTRUCTION;
  reg `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg [31:0] execute_to_memory_REGFILE_WRITE_DATA;
  reg [31:0] memory_to_writeBack_REGFILE_WRITE_DATA;
  reg [31:0] decode_to_execute_PC;
  reg [31:0] execute_to_memory_PC;
  reg [31:0] memory_to_writeBack_PC;
  reg  decode_to_execute_SRC_USE_SUB_LESS;
  reg [31:0] memory_to_writeBack_MEMORY_READ_DATA;
  reg  decode_to_execute_MEMORY_ENABLE;
  reg  execute_to_memory_MEMORY_ENABLE;
  reg  memory_to_writeBack_MEMORY_ENABLE;
  reg [31:0] decode_to_execute_RS1;
  reg [31:0] execute_to_memory_BRANCH_CALC;
  reg `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg [2:0] _zz_150_;
  `ifndef SYNTHESIS
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_1__string;
  reg [31:0] _zz_2__string;
  reg [31:0] _zz_3__string;
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_4__string;
  reg [71:0] _zz_5__string;
  reg [71:0] _zz_6__string;
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_7__string;
  reg [39:0] _zz_8__string;
  reg [39:0] _zz_9__string;
  reg [31:0] _zz_10__string;
  reg [31:0] _zz_11__string;
  reg [31:0] _zz_12__string;
  reg [31:0] _zz_13__string;
  reg [31:0] decode_ENV_CTRL_string;
  reg [31:0] _zz_14__string;
  reg [31:0] _zz_15__string;
  reg [31:0] _zz_16__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_17__string;
  reg [63:0] _zz_18__string;
  reg [63:0] _zz_19__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_22__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_24__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_30__string;
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_33__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_35__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_37__string;
  reg [39:0] _zz_45__string;
  reg [71:0] _zz_46__string;
  reg [31:0] _zz_50__string;
  reg [63:0] _zz_52__string;
  reg [31:0] _zz_53__string;
  reg [95:0] _zz_54__string;
  reg [23:0] _zz_55__string;
  reg [31:0] memory_ENV_CTRL_string;
  reg [31:0] _zz_61__string;
  reg [31:0] execute_ENV_CTRL_string;
  reg [31:0] _zz_62__string;
  reg [31:0] writeBack_ENV_CTRL_string;
  reg [31:0] _zz_65__string;
  reg [23:0] _zz_118__string;
  reg [95:0] _zz_119__string;
  reg [31:0] _zz_120__string;
  reg [63:0] _zz_121__string;
  reg [31:0] _zz_122__string;
  reg [71:0] _zz_123__string;
  reg [39:0] _zz_124__string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [31:0] decode_to_execute_ENV_CTRL_string;
  reg [31:0] execute_to_memory_ENV_CTRL_string;
  reg [31:0] memory_to_writeBack_ENV_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  `endif

  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;
  assign _zz_154_ = ((execute_arbitration_isValid && execute_LightShifterPlugin_isShift) && (execute_SRC2[4 : 0] != (5'b00000)));
  assign _zz_155_ = (! execute_arbitration_isStuckByOthers);
  assign _zz_156_ = (execute_arbitration_isValid && execute_DO_EBREAK);
  assign _zz_157_ = (({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00)) == 1'b0);
  assign _zz_158_ = (DebugPlugin_stepIt && _zz_76_);
  assign _zz_159_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_160_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_161_ = (IBusSimplePlugin_fetchPc_preOutput_valid && IBusSimplePlugin_fetchPc_preOutput_ready);
  assign _zz_162_ = debug_bus_cmd_payload_address[7 : 2];
  assign _zz_163_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_164_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_165_ = execute_INSTRUCTION[13];
  assign _zz_166_ = (_zz_84_ & (~ _zz_167_));
  assign _zz_167_ = (_zz_84_ - (2'b01));
  assign _zz_168_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_169_ = {29'd0, _zz_168_};
  assign _zz_170_ = (IBusSimplePlugin_pendingCmd + _zz_172_);
  assign _zz_171_ = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign _zz_172_ = {2'd0, _zz_171_};
  assign _zz_173_ = iBus_rsp_valid;
  assign _zz_174_ = {2'd0, _zz_173_};
  assign _zz_175_ = (iBus_rsp_valid && (IBusSimplePlugin_rspJoin_discardCounter != (3'b000)));
  assign _zz_176_ = {2'd0, _zz_175_};
  assign _zz_177_ = _zz_111_[0 : 0];
  assign _zz_178_ = _zz_111_[1 : 1];
  assign _zz_179_ = _zz_111_[2 : 2];
  assign _zz_180_ = _zz_111_[3 : 3];
  assign _zz_181_ = _zz_111_[12 : 12];
  assign _zz_182_ = _zz_111_[15 : 15];
  assign _zz_183_ = _zz_111_[16 : 16];
  assign _zz_184_ = _zz_111_[17 : 17];
  assign _zz_185_ = _zz_111_[22 : 22];
  assign _zz_186_ = _zz_111_[23 : 23];
  assign _zz_187_ = execute_SRC_LESS;
  assign _zz_188_ = (3'b100);
  assign _zz_189_ = decode_INSTRUCTION[19 : 15];
  assign _zz_190_ = decode_INSTRUCTION[31 : 20];
  assign _zz_191_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_192_ = ($signed(_zz_193_) + $signed(_zz_197_));
  assign _zz_193_ = ($signed(_zz_194_) + $signed(_zz_195_));
  assign _zz_194_ = execute_SRC1;
  assign _zz_195_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_196_ = (execute_SRC_USE_SUB_LESS ? _zz_198_ : _zz_199_);
  assign _zz_197_ = {{30{_zz_196_[1]}}, _zz_196_};
  assign _zz_198_ = (2'b01);
  assign _zz_199_ = (2'b00);
  assign _zz_200_ = (_zz_201_ >>> 1);
  assign _zz_201_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_LightShifterPlugin_shiftInput[31]),execute_LightShifterPlugin_shiftInput};
  assign _zz_202_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_203_ = execute_INSTRUCTION[31 : 20];
  assign _zz_204_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_205_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_206_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_207_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_208_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_209_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_210_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_211_ = 1'b1;
  assign _zz_212_ = 1'b1;
  assign _zz_213_ = (32'b00000000000000000000000000010000);
  assign _zz_214_ = ((decode_INSTRUCTION & (32'b00000000000000000001000001010000)) == (32'b00000000000000000001000001010000));
  assign _zz_215_ = ((decode_INSTRUCTION & (32'b00000000000000000010000001010000)) == (32'b00000000000000000010000001010000));
  assign _zz_216_ = ((decode_INSTRUCTION & _zz_223_) == (32'b00000000000000000001000000000000));
  assign _zz_217_ = _zz_114_;
  assign _zz_218_ = {_zz_114_,{_zz_224_,_zz_225_}};
  assign _zz_219_ = (3'b000);
  assign _zz_220_ = ((_zz_226_ == _zz_227_) != (1'b0));
  assign _zz_221_ = ({_zz_228_,_zz_229_} != (2'b00));
  assign _zz_222_ = {(_zz_230_ != _zz_231_),{_zz_232_,{_zz_233_,_zz_234_}}};
  assign _zz_223_ = (32'b00000000000000000001000000000000);
  assign _zz_224_ = ((decode_INSTRUCTION & (32'b00000000000000000011000000000000)) == (32'b00000000000000000001000000000000));
  assign _zz_225_ = ((decode_INSTRUCTION & (32'b00000000000000000011000000000000)) == (32'b00000000000000000010000000000000));
  assign _zz_226_ = (decode_INSTRUCTION & (32'b00000000000000000111000001010100));
  assign _zz_227_ = (32'b00000000000000000101000000010000);
  assign _zz_228_ = ((decode_INSTRUCTION & _zz_235_) == (32'b01000000000000000001000000010000));
  assign _zz_229_ = ((decode_INSTRUCTION & _zz_236_) == (32'b00000000000000000001000000010000));
  assign _zz_230_ = {_zz_114_,{_zz_113_,{_zz_237_,_zz_238_}}};
  assign _zz_231_ = (5'b00000);
  assign _zz_232_ = (_zz_117_ != (1'b0));
  assign _zz_233_ = (_zz_239_ != (1'b0));
  assign _zz_234_ = {(_zz_240_ != _zz_241_),{_zz_242_,{_zz_243_,_zz_244_}}};
  assign _zz_235_ = (32'b01000000000000000011000001010100);
  assign _zz_236_ = (32'b00000000000000000111000001010100);
  assign _zz_237_ = ((decode_INSTRUCTION & _zz_245_) == (32'b00000000000000000001000000010000));
  assign _zz_238_ = {(_zz_246_ == _zz_247_),_zz_117_};
  assign _zz_239_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000000000));
  assign _zz_240_ = _zz_116_;
  assign _zz_241_ = (1'b0);
  assign _zz_242_ = ((_zz_248_ == _zz_249_) != (1'b0));
  assign _zz_243_ = ({_zz_250_,_zz_251_} != (3'b000));
  assign _zz_244_ = {(_zz_252_ != _zz_253_),{_zz_254_,{_zz_255_,_zz_256_}}};
  assign _zz_245_ = (32'b00000000000000000001000000010000);
  assign _zz_246_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010000));
  assign _zz_247_ = (32'b00000000000000000010000000010000);
  assign _zz_248_ = (decode_INSTRUCTION & (32'b00000000000000000000000001011000));
  assign _zz_249_ = (32'b00000000000000000000000001000000);
  assign _zz_250_ = ((decode_INSTRUCTION & _zz_257_) == (32'b00000000000000000000000001000000));
  assign _zz_251_ = {(_zz_258_ == _zz_259_),(_zz_260_ == _zz_261_)};
  assign _zz_252_ = {(_zz_262_ == _zz_263_),{_zz_264_,_zz_265_}};
  assign _zz_253_ = (3'b000);
  assign _zz_254_ = (_zz_112_ != (1'b0));
  assign _zz_255_ = (_zz_266_ != (1'b0));
  assign _zz_256_ = {(_zz_267_ != _zz_268_),{_zz_269_,{_zz_270_,_zz_271_}}};
  assign _zz_257_ = (32'b00000000000000000000000001000100);
  assign _zz_258_ = (decode_INSTRUCTION & (32'b01000000000000000000000000110000));
  assign _zz_259_ = (32'b01000000000000000000000000110000);
  assign _zz_260_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010100));
  assign _zz_261_ = (32'b00000000000000000010000000010000);
  assign _zz_262_ = (decode_INSTRUCTION & (32'b00000000000000000100000000000100));
  assign _zz_263_ = (32'b00000000000000000100000000000000);
  assign _zz_264_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001100100)) == (32'b00000000000000000000000000100100));
  assign _zz_265_ = ((decode_INSTRUCTION & (32'b00000000000000000011000000000100)) == (32'b00000000000000000001000000000000));
  assign _zz_266_ = ((decode_INSTRUCTION & (32'b00000000000100000011000001010000)) == (32'b00000000000000000000000001010000));
  assign _zz_267_ = {(_zz_272_ == _zz_273_),(_zz_274_ == _zz_275_)};
  assign _zz_268_ = (2'b00);
  assign _zz_269_ = ({_zz_116_,_zz_115_} != (2'b00));
  assign _zz_270_ = ({_zz_276_,_zz_277_} != (2'b00));
  assign _zz_271_ = {(_zz_278_ != _zz_279_),{_zz_280_,{_zz_281_,_zz_282_}}};
  assign _zz_272_ = (decode_INSTRUCTION & (32'b00000000000000000000000001010000));
  assign _zz_273_ = (32'b00000000000000000000000001000000);
  assign _zz_274_ = (decode_INSTRUCTION & (32'b00000000000100000011000001000000));
  assign _zz_275_ = (32'b00000000000000000000000001000000);
  assign _zz_276_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000100));
  assign _zz_277_ = _zz_115_;
  assign _zz_278_ = {_zz_114_,((decode_INSTRUCTION & _zz_283_) == (32'b00000000000000000000000000100000))};
  assign _zz_279_ = (2'b00);
  assign _zz_280_ = ({_zz_114_,_zz_113_} != (2'b00));
  assign _zz_281_ = ({_zz_284_,{_zz_285_,_zz_286_}} != (4'b0000));
  assign _zz_282_ = {({_zz_287_,_zz_288_} != (2'b00)),{(_zz_289_ != _zz_290_),(_zz_291_ != _zz_292_)}};
  assign _zz_283_ = (32'b00000000000000000000000001110000);
  assign _zz_284_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000000));
  assign _zz_285_ = ((decode_INSTRUCTION & _zz_293_) == (32'b00000000000000000000000000000000));
  assign _zz_286_ = {_zz_112_,(_zz_294_ == _zz_295_)};
  assign _zz_287_ = ((decode_INSTRUCTION & _zz_296_) == (32'b00000000000000000010000000000000));
  assign _zz_288_ = ((decode_INSTRUCTION & _zz_297_) == (32'b00000000000000000001000000000000));
  assign _zz_289_ = {(_zz_298_ == _zz_299_),(_zz_300_ == _zz_301_)};
  assign _zz_290_ = (2'b00);
  assign _zz_291_ = ((decode_INSTRUCTION & _zz_302_) == (32'b00000000000000000000000001010000));
  assign _zz_292_ = (1'b0);
  assign _zz_293_ = (32'b00000000000000000000000000011000);
  assign _zz_294_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000100));
  assign _zz_295_ = (32'b00000000000000000001000000000000);
  assign _zz_296_ = (32'b00000000000000000010000000010000);
  assign _zz_297_ = (32'b00000000000000000101000000000000);
  assign _zz_298_ = (decode_INSTRUCTION & (32'b00000000000000000000000000110100));
  assign _zz_299_ = (32'b00000000000000000000000000100000);
  assign _zz_300_ = (decode_INSTRUCTION & (32'b00000000000000000000000001100100));
  assign _zz_301_ = (32'b00000000000000000000000000100000);
  assign _zz_302_ = (32'b00010000000000000011000001010000);
  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_40_) begin
      RegFilePlugin_regFile[writeBack_RegFilePlugin_regFileWrite_payload_address] <= writeBack_RegFilePlugin_regFileWrite_payload_data;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_211_) begin
      _zz_152_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_212_) begin
      _zz_153_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid(iBus_rsp_takeWhen_valid),
    .io_push_ready(IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready),
    .io_push_payload_error(iBus_rsp_takeWhen_payload_error),
    .io_push_payload_inst(iBus_rsp_takeWhen_payload_inst),
    .io_pop_valid(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid),
    .io_pop_ready(IBusSimplePlugin_rspJoin_rspBufferOutput_ready),
    .io_pop_payload_error(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error),
    .io_pop_payload_inst(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst),
    .io_flush(_zz_151_),
    .io_occupancy(IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(decode_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_BRANCH_CTRL_string = "JALR";
      default : decode_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_1__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_1__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_1__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_1__string = "JALR";
      default : _zz_1__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_2_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_2__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_2__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_2__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_2__string = "JALR";
      default : _zz_2__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_3__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_3__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_3__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_3__string = "JALR";
      default : _zz_3__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_4_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_4__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_4__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_4__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_4__string = "SRA_1    ";
      default : _zz_4__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_5__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_5__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_5__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_5__string = "SRA_1    ";
      default : _zz_5__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_6_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_6__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_6__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_6__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_6__string = "SRA_1    ";
      default : _zz_6__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : decode_ALU_BITWISE_CTRL_string = "SRC1 ";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_7_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_7__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_7__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_7__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_7__string = "SRC1 ";
      default : _zz_7__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_8_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_8__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_8__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_8__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_8__string = "SRC1 ";
      default : _zz_8__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_9__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_9__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_9__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_9__string = "SRC1 ";
      default : _zz_9__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_10_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_10__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_10__string = "XRET";
      default : _zz_10__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_11_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_11__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_11__string = "XRET";
      default : _zz_11__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_12_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_12__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_12__string = "XRET";
      default : _zz_12__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_13_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_13__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_13__string = "XRET";
      default : _zz_13__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_ENV_CTRL_string = "XRET";
      default : decode_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_14_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_14__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_14__string = "XRET";
      default : _zz_14__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_15_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_15__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_15__string = "XRET";
      default : _zz_15__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_16_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_16__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_16__string = "XRET";
      default : _zz_16__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_ALU_CTRL_string = "BITWISE ";
      default : decode_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_17_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_17__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_17__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_17__string = "BITWISE ";
      default : _zz_17__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_18_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_18__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_18__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_18__string = "BITWISE ";
      default : _zz_18__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_19_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_19__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_19__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_19__string = "BITWISE ";
      default : _zz_19__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : execute_BRANCH_CTRL_string = "JALR";
      default : execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_22_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_22__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_22__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_22__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_22__string = "JALR";
      default : _zz_22__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_24_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_24__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_24__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_24__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_24__string = "SRA_1    ";
      default : _zz_24__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_SRC2_CTRL_string = "PC ";
      default : decode_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_30_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_30__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_30__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_30__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_30__string = "PC ";
      default : _zz_30__string = "???";
    endcase
  end
  always @(*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_SRC1_CTRL_string = "URS1        ";
      default : decode_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_33_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_33__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_33__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_33__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_33__string = "URS1        ";
      default : _zz_33__string = "????????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : execute_ALU_CTRL_string = "BITWISE ";
      default : execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_35_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_35__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_35__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_35__string = "BITWISE ";
      default : _zz_35__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : execute_ALU_BITWISE_CTRL_string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : execute_ALU_BITWISE_CTRL_string = "SRC1 ";
      default : execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_37_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_37__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_37__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_37__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_37__string = "SRC1 ";
      default : _zz_37__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_45_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_45__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_45__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_45__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_45__string = "SRC1 ";
      default : _zz_45__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_46_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_46__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_46__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_46__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_46__string = "SRA_1    ";
      default : _zz_46__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_50_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_50__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_50__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_50__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_50__string = "JALR";
      default : _zz_50__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_52_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_52__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_52__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_52__string = "BITWISE ";
      default : _zz_52__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_53_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_53__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_53__string = "XRET";
      default : _zz_53__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_54_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_54__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_54__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_54__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_54__string = "URS1        ";
      default : _zz_54__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_55_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_55__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_55__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_55__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_55__string = "PC ";
      default : _zz_55__string = "???";
    endcase
  end
  always @(*) begin
    case(memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_ENV_CTRL_string = "XRET";
      default : memory_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_61_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_61__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_61__string = "XRET";
      default : _zz_61__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_ENV_CTRL_string = "XRET";
      default : execute_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_62_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_62__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_62__string = "XRET";
      default : _zz_62__string = "????";
    endcase
  end
  always @(*) begin
    case(writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : writeBack_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : writeBack_ENV_CTRL_string = "XRET";
      default : writeBack_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_65_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_65__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_65__string = "XRET";
      default : _zz_65__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_118_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_118__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_118__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_118__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_118__string = "PC ";
      default : _zz_118__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_119_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_119__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_119__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_119__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_119__string = "URS1        ";
      default : _zz_119__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_120_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_120__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_120__string = "XRET";
      default : _zz_120__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_121_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_121__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_121__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_121__string = "BITWISE ";
      default : _zz_121__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_122_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_122__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_122__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_122__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_122__string = "JALR";
      default : _zz_122__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_123_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_123__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_123__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_123__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_123__string = "SRA_1    ";
      default : _zz_123__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_124_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_124__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_124__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_124__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_124__string = "SRC1 ";
      default : _zz_124__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_to_execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_to_execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_to_execute_ALU_CTRL_string = "BITWISE ";
      default : decode_to_execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_to_execute_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_to_execute_ENV_CTRL_string = "XRET";
      default : decode_to_execute_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_to_memory_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_to_memory_ENV_CTRL_string = "XRET";
      default : execute_to_memory_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(memory_to_writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_to_writeBack_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_to_writeBack_ENV_CTRL_string = "XRET";
      default : memory_to_writeBack_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : decode_to_execute_ALU_BITWISE_CTRL_string = "SRC1 ";
      default : decode_to_execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_to_execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_to_execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_to_execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_to_execute_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_to_execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_to_execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_to_execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_to_execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_to_execute_BRANCH_CTRL_string = "JALR";
      default : decode_to_execute_BRANCH_CTRL_string = "????";
    endcase
  end
  `endif

  assign decode_BRANCH_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign execute_BRANCH_CALC = _zz_21_;
  assign decode_RS1 = _zz_42_;
  assign decode_MEMORY_ENABLE = _zz_49_;
  assign memory_MEMORY_READ_DATA = _zz_67_;
  assign decode_SRC_USE_SUB_LESS = _zz_51_;
  assign memory_PC = execute_to_memory_PC;
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_36_;
  assign decode_SHIFT_CTRL = _zz_4_;
  assign _zz_5_ = _zz_6_;
  assign decode_SRC1 = _zz_34_;
  assign decode_SRC2 = _zz_31_;
  assign decode_IS_CSR = _zz_44_;
  assign decode_ALU_BITWISE_CTRL = _zz_7_;
  assign _zz_8_ = _zz_9_;
  assign decode_SRC_LESS_UNSIGNED = _zz_57_;
  assign decode_RS2 = _zz_41_;
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_70_;
  assign decode_CSR_READ_OPCODE = _zz_63_;
  assign execute_BRANCH_DO = _zz_23_;
  assign decode_DO_EBREAK = _zz_20_;
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_43_;
  assign _zz_10_ = _zz_11_;
  assign _zz_12_ = _zz_13_;
  assign decode_ENV_CTRL = _zz_14_;
  assign _zz_15_ = _zz_16_;
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = _zz_68_;
  assign decode_CSR_WRITE_OPCODE = _zz_64_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_48_;
  assign decode_ALU_CTRL = _zz_17_;
  assign _zz_18_ = _zz_19_;
  assign execute_DO_EBREAK = decode_to_execute_DO_EBREAK;
  assign decode_IS_EBREAK = _zz_59_;
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_PC = decode_to_execute_PC;
  assign execute_RS1 = decode_to_execute_RS1;
  assign execute_BRANCH_CTRL = _zz_22_;
  assign decode_RS2_USE = _zz_58_;
  assign decode_RS1_USE = _zz_56_;
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign execute_SHIFT_CTRL = _zz_24_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_28_ = decode_PC;
  assign _zz_29_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_30_;
  assign _zz_32_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_33_;
  assign execute_SRC_ADD_SUB = _zz_27_;
  assign execute_SRC_LESS = _zz_25_;
  assign execute_ALU_CTRL = _zz_35_;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign execute_ALU_BITWISE_CTRL = _zz_37_;
  assign _zz_38_ = writeBack_INSTRUCTION;
  assign _zz_39_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_40_ = 1'b0;
    if(writeBack_RegFilePlugin_regFileWrite_valid)begin
      _zz_40_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = _zz_73_;
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_47_;
    if((decode_INSTRUCTION[11 : 7] == (5'b00000)))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  always @ (*) begin
    _zz_60_ = execute_REGFILE_WRITE_DATA;
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_ALIGNEMENT_FAULT)) && (! execute_DBusSimplePlugin_cmdSent)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if((execute_arbitration_isValid && execute_IS_CSR))begin
      _zz_60_ = execute_CsrPlugin_readData;
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
    if(_zz_154_)begin
      _zz_60_ = _zz_133_;
      if(_zz_155_)begin
        if(! execute_LightShifterPlugin_done) begin
          execute_arbitration_haltItself = 1'b1;
        end
      end
    end
  end

  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_61_;
  assign execute_ENV_CTRL = _zz_62_;
  assign writeBack_ENV_CTRL = _zz_65_;
  always @ (*) begin
    _zz_66_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_66_ = writeBack_DBusSimplePlugin_rspFormated;
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_SRC_ADD = _zz_26_;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_ALIGNEMENT_FAULT = 1'b0;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  always @ (*) begin
    _zz_69_ = memory_FORMAL_PC_NEXT;
    if(_zz_80_)begin
      _zz_69_ = _zz_81_;
    end
  end

  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  assign decode_PC = _zz_72_;
  assign decode_INSTRUCTION = _zz_71_;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    decode_arbitration_isValid = (IBusSimplePlugin_injector_decodeInput_valid && (! IBusSimplePlugin_injector_decodeRemoved));
    _zz_83_ = 1'b0;
    case(_zz_150_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_isValid = 1'b1;
        decode_arbitration_haltItself = 1'b1;
      end
      3'b011 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b100 : begin
        _zz_83_ = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((CsrPlugin_interrupt && decode_arbitration_isValid))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(({(memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),(execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))} != (2'b00)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if((decode_arbitration_isValid && (_zz_134_ || _zz_135_)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_flushAll = 1'b0;
  assign decode_arbitration_redoIt = 1'b0;
  always @ (*) begin
    execute_arbitration_haltByOther = 1'b0;
    _zz_74_ = 1'b0;
    _zz_75_ = 1'b0;
    if(_zz_156_)begin
      execute_arbitration_haltByOther = 1'b1;
      if(_zz_157_)begin
        _zz_75_ = 1'b1;
        _zz_74_ = 1'b1;
      end
    end
    if(DebugPlugin_haltIt)begin
      _zz_74_ = 1'b1;
    end
    if(_zz_158_)begin
      _zz_74_ = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_flushAll = 1'b0;
    if(_zz_80_)begin
      execute_arbitration_flushAll = 1'b1;
    end
    if(_zz_156_)begin
      if(_zz_157_)begin
        execute_arbitration_flushAll = 1'b1;
      end
    end
  end

  assign execute_arbitration_redoIt = 1'b0;
  always @ (*) begin
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_INSTRUCTION[5])) && (! dBus_rsp_ready)))begin
      memory_arbitration_haltItself = 1'b1;
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushAll = 1'b0;
    _zz_77_ = 1'b0;
    _zz_78_ = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    if(_zz_159_)begin
      _zz_77_ = 1'b1;
      _zz_78_ = {CsrPlugin_mtvec_base,(2'b00)};
      memory_arbitration_flushAll = 1'b1;
    end
    if(_zz_160_)begin
      _zz_78_ = CsrPlugin_mepc;
      _zz_77_ = 1'b1;
      memory_arbitration_flushAll = 1'b1;
    end
  end

  assign memory_arbitration_redoIt = 1'b0;
  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushAll = 1'b0;
  assign writeBack_arbitration_redoIt = 1'b0;
  always @ (*) begin
    _zz_76_ = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid || IBusSimplePlugin_iBusRsp_stages_2_input_valid))begin
      _zz_76_ = 1'b1;
    end
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      _zz_76_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_79_ = 1'b1;
    if((DebugPlugin_haltIt || DebugPlugin_stepIt))begin
      _zz_79_ = 1'b0;
    end
  end

  assign IBusSimplePlugin_jump_pcLoad_valid = ({_zz_80_,_zz_77_} != (2'b00));
  assign _zz_84_ = {_zz_80_,_zz_77_};
  assign IBusSimplePlugin_jump_pcLoad_payload = (_zz_166_[0] ? _zz_78_ : _zz_81_);
  assign _zz_85_ = (! _zz_74_);
  assign IBusSimplePlugin_fetchPc_output_valid = (IBusSimplePlugin_fetchPc_preOutput_valid && _zz_85_);
  assign IBusSimplePlugin_fetchPc_preOutput_ready = (IBusSimplePlugin_fetchPc_output_ready && _zz_85_);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_preOutput_payload;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_propagatePc = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_iBusRsp_stages_1_input_ready))begin
      IBusSimplePlugin_fetchPc_propagatePc = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_169_);
    IBusSimplePlugin_fetchPc_samplePcNext = 1'b0;
    if(IBusSimplePlugin_fetchPc_propagatePc)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    if(_zz_161_)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
    IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
  end

  assign IBusSimplePlugin_fetchPc_preOutput_valid = _zz_86_;
  assign IBusSimplePlugin_fetchPc_preOutput_payload = IBusSimplePlugin_fetchPc_pc;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_inputSample = 1'b1;
  assign IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
  assign _zz_87_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_87_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_87_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && ((! IBusSimplePlugin_cmd_valid) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b1;
    end
  end

  assign _zz_88_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_88_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_88_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_2_halt = 1'b0;
  assign _zz_89_ = (! IBusSimplePlugin_iBusRsp_stages_2_halt);
  assign IBusSimplePlugin_iBusRsp_stages_2_input_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_ready && _zz_89_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_valid = (IBusSimplePlugin_iBusRsp_stages_2_input_valid && _zz_89_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_payload = IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = _zz_90_;
  assign _zz_90_ = ((1'b0 && (! _zz_91_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_91_ = _zz_92_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_91_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = IBusSimplePlugin_fetchPc_pcReg;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = ((1'b0 && (! _zz_93_)) || IBusSimplePlugin_iBusRsp_stages_2_input_ready);
  assign _zz_93_ = _zz_94_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_valid = _zz_93_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_payload = _zz_95_;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_96_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_97_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_98_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_99_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_100_;
  assign _zz_73_ = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw);
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  assign _zz_72_ = IBusSimplePlugin_injector_decodeInput_payload_pc;
  assign _zz_71_ = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  assign _zz_70_ = (decode_PC + (32'b00000000000000000000000000000100));
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pendingCmdNext = (_zz_170_ - _zz_174_);
  assign IBusSimplePlugin_cmd_valid = ((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_iBusRsp_stages_1_output_ready) && (IBusSimplePlugin_pendingCmd != (3'b111)));
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_iBusRsp_stages_1_input_payload[31 : 2],(2'b00)};
  assign iBus_rsp_takeWhen_valid = (iBus_rsp_valid && (! (IBusSimplePlugin_rspJoin_discardCounter != (3'b000))));
  assign iBus_rsp_takeWhen_payload_error = iBus_rsp_payload_error;
  assign iBus_rsp_takeWhen_payload_inst = iBus_rsp_payload_inst;
  assign _zz_151_ = (IBusSimplePlugin_jump_pcLoad_valid || _zz_75_);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_valid = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBufferOutput_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  assign IBusSimplePlugin_rspJoin_issueDetected = 1'b0;
  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_2_output_valid && IBusSimplePlugin_rspJoin_rspBufferOutput_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_2_output_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_101_ = (! IBusSimplePlugin_rspJoin_issueDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_inputBeforeStage_ready && _zz_101_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_101_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  assign execute_DBusSimplePlugin_cmdSent = 1'b0;
  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_ALIGNEMENT_FAULT)) && (! execute_DBusSimplePlugin_cmdSent));
  assign dBus_cmd_payload_wr = execute_INSTRUCTION[5];
  assign dBus_cmd_payload_address = execute_SRC_ADD;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_102_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_102_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_102_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_102_;
  assign _zz_68_ = dBus_cmd_payload_address[1 : 0];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_103_ = (4'b0001);
      end
      2'b01 : begin
        _zz_103_ = (4'b0011);
      end
      default : begin
        _zz_103_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_103_ <<< dBus_cmd_payload_address[1 : 0]);
  assign _zz_67_ = dBus_rsp_data;
  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_104_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_105_[31] = _zz_104_;
    _zz_105_[30] = _zz_104_;
    _zz_105_[29] = _zz_104_;
    _zz_105_[28] = _zz_104_;
    _zz_105_[27] = _zz_104_;
    _zz_105_[26] = _zz_104_;
    _zz_105_[25] = _zz_104_;
    _zz_105_[24] = _zz_104_;
    _zz_105_[23] = _zz_104_;
    _zz_105_[22] = _zz_104_;
    _zz_105_[21] = _zz_104_;
    _zz_105_[20] = _zz_104_;
    _zz_105_[19] = _zz_104_;
    _zz_105_[18] = _zz_104_;
    _zz_105_[17] = _zz_104_;
    _zz_105_[16] = _zz_104_;
    _zz_105_[15] = _zz_104_;
    _zz_105_[14] = _zz_104_;
    _zz_105_[13] = _zz_104_;
    _zz_105_[12] = _zz_104_;
    _zz_105_[11] = _zz_104_;
    _zz_105_[10] = _zz_104_;
    _zz_105_[9] = _zz_104_;
    _zz_105_[8] = _zz_104_;
    _zz_105_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_106_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_107_[31] = _zz_106_;
    _zz_107_[30] = _zz_106_;
    _zz_107_[29] = _zz_106_;
    _zz_107_[28] = _zz_106_;
    _zz_107_[27] = _zz_106_;
    _zz_107_[26] = _zz_106_;
    _zz_107_[25] = _zz_106_;
    _zz_107_[24] = _zz_106_;
    _zz_107_[23] = _zz_106_;
    _zz_107_[22] = _zz_106_;
    _zz_107_[21] = _zz_106_;
    _zz_107_[20] = _zz_106_;
    _zz_107_[19] = _zz_106_;
    _zz_107_[18] = _zz_106_;
    _zz_107_[17] = _zz_106_;
    _zz_107_[16] = _zz_106_;
    _zz_107_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_163_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_105_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_107_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = (26'b00000000000000000001000010);
  assign CsrPlugin_mtvec_mode = (2'b00);
  assign CsrPlugin_mtvec_base = (30'b100000000000000000000000001000);
  assign CsrPlugin_medeleg = (32'b00000000000000000000000000000000);
  assign CsrPlugin_mideleg = (32'b00000000000000000000000000000000);
  assign _zz_108_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_109_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_110_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  always @ (*) begin
    CsrPlugin_interrupt = 1'b0;
    CsrPlugin_interruptCode = (4'bxxxx);
    if(CsrPlugin_mstatus_MIE)begin
      if(({_zz_110_,{_zz_109_,_zz_108_}} != (3'b000)))begin
        CsrPlugin_interrupt = 1'b1;
      end
      if(_zz_108_)begin
        CsrPlugin_interruptCode = (4'b0111);
      end
      if(_zz_109_)begin
        CsrPlugin_interruptCode = (4'b0011);
      end
      if(_zz_110_)begin
        CsrPlugin_interruptCode = (4'b1011);
      end
    end
    if((! _zz_79_))begin
      CsrPlugin_interrupt = 1'b0;
    end
  end

  assign CsrPlugin_interruptTargetPrivilege = (2'b11);
  assign CsrPlugin_exception = 1'b0;
  assign CsrPlugin_lastStageWasWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = ((! ({writeBack_arbitration_isValid,{memory_arbitration_isValid,execute_arbitration_isValid}} != (3'b000))) && IBusSimplePlugin_injector_nextPcCalc_0);
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = (CsrPlugin_interrupt && CsrPlugin_pipelineLiberator_done);
  assign CsrPlugin_targetPrivilege = CsrPlugin_interruptTargetPrivilege;
  assign CsrPlugin_trapCause = CsrPlugin_interruptCode;
  assign contextSwitching = _zz_77_;
  assign _zz_64_ = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == (5'b00000))) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == (5'b00000)))));
  assign _zz_63_ = (decode_INSTRUCTION[13 : 7] != (7'b0100000));
  assign execute_CsrPlugin_blockedBySideEffects = ({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00));
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = 1'b1;
    execute_CsrPlugin_readData = (32'b00000000000000000000000000000000);
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[12 : 11] = CsrPlugin_mstatus_MPP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mstatus_MPIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mstatus_MIE;
      end
      12'b001101000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mip_MEIP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mip_MTIP;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mip_MSIP;
      end
      12'b001100000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mie_MEIE;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mie_MTIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mie_MSIE;
      end
      12'b001101000010 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
        execute_CsrPlugin_readData[31 : 31] = CsrPlugin_mcause_interrupt;
        execute_CsrPlugin_readData[3 : 0] = CsrPlugin_mcause_exceptionCode;
      end
      default : begin
      end
    endcase
    if((CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]))begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
    if(((! execute_arbitration_isValid) || (! execute_IS_CSR)))begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((execute_INSTRUCTION[29 : 28] != CsrPlugin_privilege))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  assign execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
  assign execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
  assign execute_CsrPlugin_writeEnable = ((execute_CsrPlugin_writeInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readEnable = ((execute_CsrPlugin_readInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  always @ (*) begin
    case(_zz_165_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readData & (~ execute_SRC1)) : (execute_CsrPlugin_readData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign _zz_112_ = ((decode_INSTRUCTION & (32'b00000000000000000110000000000100)) == (32'b00000000000000000010000000000000));
  assign _zz_113_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000100000)) == (32'b00000000000000000000000000000000));
  assign _zz_114_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000000100)) == (32'b00000000000000000000000000000100));
  assign _zz_115_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001010000)) == (32'b00000000000000000100000001010000));
  assign _zz_116_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000010100)) == (32'b00000000000000000000000000000100));
  assign _zz_117_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_111_ = {(((decode_INSTRUCTION & _zz_213_) == (32'b00000000000000000000000000010000)) != (1'b0)),{({_zz_214_,_zz_215_} != (2'b00)),{({_zz_216_,_zz_217_} != (2'b00)),{(_zz_218_ != _zz_219_),{_zz_220_,{_zz_221_,_zz_222_}}}}}};
  assign _zz_59_ = _zz_177_[0];
  assign _zz_58_ = _zz_178_[0];
  assign _zz_57_ = _zz_179_[0];
  assign _zz_56_ = _zz_180_[0];
  assign _zz_118_ = _zz_111_[5 : 4];
  assign _zz_55_ = _zz_118_;
  assign _zz_119_ = _zz_111_[7 : 6];
  assign _zz_54_ = _zz_119_;
  assign _zz_120_ = _zz_111_[9 : 9];
  assign _zz_53_ = _zz_120_;
  assign _zz_121_ = _zz_111_[11 : 10];
  assign _zz_52_ = _zz_121_;
  assign _zz_51_ = _zz_181_[0];
  assign _zz_122_ = _zz_111_[14 : 13];
  assign _zz_50_ = _zz_122_;
  assign _zz_49_ = _zz_182_[0];
  assign _zz_48_ = _zz_183_[0];
  assign _zz_47_ = _zz_184_[0];
  assign _zz_123_ = _zz_111_[19 : 18];
  assign _zz_46_ = _zz_123_;
  assign _zz_124_ = _zz_111_[21 : 20];
  assign _zz_45_ = _zz_124_;
  assign _zz_44_ = _zz_185_[0];
  assign _zz_43_ = _zz_186_[0];
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_152_;
  assign decode_RegFilePlugin_rs2Data = _zz_153_;
  assign _zz_42_ = decode_RegFilePlugin_rs1Data;
  assign _zz_41_ = decode_RegFilePlugin_rs2Data;
  always @ (*) begin
    writeBack_RegFilePlugin_regFileWrite_valid = (_zz_39_ && writeBack_arbitration_isFiring);
    if(_zz_125_)begin
      writeBack_RegFilePlugin_regFileWrite_valid = 1'b1;
    end
  end

  assign writeBack_RegFilePlugin_regFileWrite_payload_address = _zz_38_[11 : 7];
  assign writeBack_RegFilePlugin_regFileWrite_payload_data = _zz_66_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = execute_SRC1;
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_126_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_126_ = {31'd0, _zz_187_};
      end
      default : begin
        _zz_126_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  assign _zz_36_ = _zz_126_;
  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_127_ = _zz_32_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_127_ = {29'd0, _zz_188_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_127_ = {decode_INSTRUCTION[31 : 12],(12'b000000000000)};
      end
      default : begin
        _zz_127_ = {27'd0, _zz_189_};
      end
    endcase
  end

  assign _zz_34_ = _zz_127_;
  assign _zz_128_ = _zz_190_[11];
  always @ (*) begin
    _zz_129_[19] = _zz_128_;
    _zz_129_[18] = _zz_128_;
    _zz_129_[17] = _zz_128_;
    _zz_129_[16] = _zz_128_;
    _zz_129_[15] = _zz_128_;
    _zz_129_[14] = _zz_128_;
    _zz_129_[13] = _zz_128_;
    _zz_129_[12] = _zz_128_;
    _zz_129_[11] = _zz_128_;
    _zz_129_[10] = _zz_128_;
    _zz_129_[9] = _zz_128_;
    _zz_129_[8] = _zz_128_;
    _zz_129_[7] = _zz_128_;
    _zz_129_[6] = _zz_128_;
    _zz_129_[5] = _zz_128_;
    _zz_129_[4] = _zz_128_;
    _zz_129_[3] = _zz_128_;
    _zz_129_[2] = _zz_128_;
    _zz_129_[1] = _zz_128_;
    _zz_129_[0] = _zz_128_;
  end

  assign _zz_130_ = _zz_191_[11];
  always @ (*) begin
    _zz_131_[19] = _zz_130_;
    _zz_131_[18] = _zz_130_;
    _zz_131_[17] = _zz_130_;
    _zz_131_[16] = _zz_130_;
    _zz_131_[15] = _zz_130_;
    _zz_131_[14] = _zz_130_;
    _zz_131_[13] = _zz_130_;
    _zz_131_[12] = _zz_130_;
    _zz_131_[11] = _zz_130_;
    _zz_131_[10] = _zz_130_;
    _zz_131_[9] = _zz_130_;
    _zz_131_[8] = _zz_130_;
    _zz_131_[7] = _zz_130_;
    _zz_131_[6] = _zz_130_;
    _zz_131_[5] = _zz_130_;
    _zz_131_[4] = _zz_130_;
    _zz_131_[3] = _zz_130_;
    _zz_131_[2] = _zz_130_;
    _zz_131_[1] = _zz_130_;
    _zz_131_[0] = _zz_130_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_132_ = _zz_29_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_132_ = {_zz_129_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_132_ = {_zz_131_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_132_ = _zz_28_;
      end
    endcase
  end

  assign _zz_31_ = _zz_132_;
  assign execute_SrcPlugin_addSub = _zz_192_;
  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign _zz_27_ = execute_SrcPlugin_addSub;
  assign _zz_26_ = execute_SrcPlugin_addSub;
  assign _zz_25_ = execute_SrcPlugin_less;
  assign execute_LightShifterPlugin_isShift = (execute_SHIFT_CTRL != `ShiftCtrlEnum_defaultEncoding_DISABLE_1);
  assign execute_LightShifterPlugin_amplitude = (execute_LightShifterPlugin_isActive ? execute_LightShifterPlugin_amplitudeReg : execute_SRC2[4 : 0]);
  assign execute_LightShifterPlugin_shiftInput = (execute_LightShifterPlugin_isActive ? memory_REGFILE_WRITE_DATA : execute_SRC1);
  assign execute_LightShifterPlugin_done = (execute_LightShifterPlugin_amplitude[4 : 1] == (4'b0000));
  always @ (*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
        _zz_133_ = (execute_LightShifterPlugin_shiftInput <<< 1);
      end
      default : begin
        _zz_133_ = _zz_200_;
      end
    endcase
  end

  always @ (*) begin
    _zz_134_ = 1'b0;
    _zz_135_ = 1'b0;
    if(_zz_137_)begin
      if((_zz_138_ == decode_INSTRUCTION[19 : 15]))begin
        _zz_134_ = 1'b1;
      end
      if((_zz_138_ == decode_INSTRUCTION[24 : 20]))begin
        _zz_135_ = 1'b1;
      end
    end
    if((writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID))begin
      if((1'b1 || (! 1'b1)))begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_134_ = 1'b1;
        end
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_135_ = 1'b1;
        end
      end
    end
    if((memory_arbitration_isValid && memory_REGFILE_WRITE_VALID))begin
      if((1'b1 || (! memory_BYPASSABLE_MEMORY_STAGE)))begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_134_ = 1'b1;
        end
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_135_ = 1'b1;
        end
      end
    end
    if((execute_arbitration_isValid && execute_REGFILE_WRITE_VALID))begin
      if((1'b1 || (! execute_BYPASSABLE_EXECUTE_STAGE)))begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_134_ = 1'b1;
        end
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_135_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_134_ = 1'b0;
    end
    if((! decode_RS2_USE))begin
      _zz_135_ = 1'b0;
    end
  end

  assign _zz_136_ = (_zz_39_ && writeBack_arbitration_isFiring);
  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_139_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_139_ == (3'b000))) begin
        _zz_140_ = execute_BranchPlugin_eq;
    end else if((_zz_139_ == (3'b001))) begin
        _zz_140_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_139_ & (3'b101)) == (3'b101)))) begin
        _zz_140_ = (! execute_SRC_LESS);
    end else begin
        _zz_140_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_141_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_141_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_141_ = 1'b1;
      end
      default : begin
        _zz_141_ = _zz_140_;
      end
    endcase
  end

  assign _zz_23_ = _zz_141_;
  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_142_ = _zz_202_[19];
  always @ (*) begin
    _zz_143_[10] = _zz_142_;
    _zz_143_[9] = _zz_142_;
    _zz_143_[8] = _zz_142_;
    _zz_143_[7] = _zz_142_;
    _zz_143_[6] = _zz_142_;
    _zz_143_[5] = _zz_142_;
    _zz_143_[4] = _zz_142_;
    _zz_143_[3] = _zz_142_;
    _zz_143_[2] = _zz_142_;
    _zz_143_[1] = _zz_142_;
    _zz_143_[0] = _zz_142_;
  end

  assign _zz_144_ = _zz_203_[11];
  always @ (*) begin
    _zz_145_[19] = _zz_144_;
    _zz_145_[18] = _zz_144_;
    _zz_145_[17] = _zz_144_;
    _zz_145_[16] = _zz_144_;
    _zz_145_[15] = _zz_144_;
    _zz_145_[14] = _zz_144_;
    _zz_145_[13] = _zz_144_;
    _zz_145_[12] = _zz_144_;
    _zz_145_[11] = _zz_144_;
    _zz_145_[10] = _zz_144_;
    _zz_145_[9] = _zz_144_;
    _zz_145_[8] = _zz_144_;
    _zz_145_[7] = _zz_144_;
    _zz_145_[6] = _zz_144_;
    _zz_145_[5] = _zz_144_;
    _zz_145_[4] = _zz_144_;
    _zz_145_[3] = _zz_144_;
    _zz_145_[2] = _zz_144_;
    _zz_145_[1] = _zz_144_;
    _zz_145_[0] = _zz_144_;
  end

  assign _zz_146_ = _zz_204_[11];
  always @ (*) begin
    _zz_147_[18] = _zz_146_;
    _zz_147_[17] = _zz_146_;
    _zz_147_[16] = _zz_146_;
    _zz_147_[15] = _zz_146_;
    _zz_147_[14] = _zz_146_;
    _zz_147_[13] = _zz_146_;
    _zz_147_[12] = _zz_146_;
    _zz_147_[11] = _zz_146_;
    _zz_147_[10] = _zz_146_;
    _zz_147_[9] = _zz_146_;
    _zz_147_[8] = _zz_146_;
    _zz_147_[7] = _zz_146_;
    _zz_147_[6] = _zz_146_;
    _zz_147_[5] = _zz_146_;
    _zz_147_[4] = _zz_146_;
    _zz_147_[3] = _zz_146_;
    _zz_147_[2] = _zz_146_;
    _zz_147_[1] = _zz_146_;
    _zz_147_[0] = _zz_146_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_148_ = {{_zz_143_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_148_ = {_zz_145_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_148_ = {{_zz_147_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_148_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign _zz_21_ = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign _zz_80_ = ((memory_arbitration_isValid && (! memory_arbitration_isStuckByOthers)) && memory_BRANCH_DO);
  assign _zz_81_ = memory_BRANCH_CALC;
  assign DebugPlugin_isPipBusy = (DebugPlugin_isPipActive || DebugPlugin_isPipActive_regNext);
  always @ (*) begin
    debug_bus_cmd_ready = 1'b1;
    _zz_82_ = 1'b0;
    if(debug_bus_cmd_valid)begin
      case(_zz_162_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            _zz_82_ = 1'b1;
            debug_bus_cmd_ready = _zz_83_;
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    debug_bus_rsp_data = DebugPlugin_busReadDataReg;
    if((! _zz_149_))begin
      debug_bus_rsp_data[0] = DebugPlugin_resetIt;
      debug_bus_rsp_data[1] = DebugPlugin_haltIt;
      debug_bus_rsp_data[2] = DebugPlugin_isPipBusy;
      debug_bus_rsp_data[3] = DebugPlugin_haltedByBreak;
      debug_bus_rsp_data[4] = DebugPlugin_stepIt;
    end
  end

  assign _zz_20_ = ((! DebugPlugin_haltIt) && (decode_IS_EBREAK || 1'b0));
  assign debug_resetOut = DebugPlugin_resetIt_regNext;
  assign _zz_19_ = decode_ALU_CTRL;
  assign _zz_17_ = _zz_52_;
  assign _zz_35_ = decode_to_execute_ALU_CTRL;
  assign _zz_16_ = decode_ENV_CTRL;
  assign _zz_13_ = execute_ENV_CTRL;
  assign _zz_11_ = memory_ENV_CTRL;
  assign _zz_14_ = _zz_53_;
  assign _zz_62_ = decode_to_execute_ENV_CTRL;
  assign _zz_61_ = execute_to_memory_ENV_CTRL;
  assign _zz_65_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_33_ = _zz_54_;
  assign _zz_30_ = _zz_55_;
  assign _zz_9_ = decode_ALU_BITWISE_CTRL;
  assign _zz_7_ = _zz_45_;
  assign _zz_37_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_6_ = decode_SHIFT_CTRL;
  assign _zz_4_ = _zz_46_;
  assign _zz_24_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_3_ = decode_BRANCH_CTRL;
  assign _zz_1_ = _zz_50_;
  assign _zz_22_ = decode_to_execute_BRANCH_CTRL;
  assign decode_arbitration_isFlushed = ({writeBack_arbitration_flushAll,{memory_arbitration_flushAll,{execute_arbitration_flushAll,decode_arbitration_flushAll}}} != (4'b0000));
  assign execute_arbitration_isFlushed = ({writeBack_arbitration_flushAll,{memory_arbitration_flushAll,execute_arbitration_flushAll}} != (3'b000));
  assign memory_arbitration_isFlushed = ({writeBack_arbitration_flushAll,memory_arbitration_flushAll} != (2'b00));
  assign writeBack_arbitration_isFlushed = (writeBack_arbitration_flushAll != (1'b0));
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      CsrPlugin_privilege <= (2'b11);
      IBusSimplePlugin_fetchPc_pcReg <= (32'b10000000000000000000000000000000);
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      _zz_86_ <= 1'b0;
      _zz_92_ <= 1'b0;
      _zz_94_ <= 1'b0;
      _zz_96_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_3 <= 1'b0;
      IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      IBusSimplePlugin_pendingCmd <= (3'b000);
      IBusSimplePlugin_rspJoin_discardCounter <= (3'b000);
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mip_MEIP <= 1'b0;
      CsrPlugin_mip_MTIP <= 1'b0;
      CsrPlugin_mip_MSIP <= 1'b0;
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      _zz_125_ <= 1'b1;
      execute_LightShifterPlugin_isActive <= 1'b0;
      _zz_137_ <= 1'b0;
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      _zz_150_ <= (3'b000);
      memory_to_writeBack_REGFILE_WRITE_DATA <= (32'b00000000000000000000000000000000);
      memory_to_writeBack_INSTRUCTION <= (32'b00000000000000000000000000000000);
    end else begin
      if(IBusSimplePlugin_fetchPc_propagatePc)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(IBusSimplePlugin_jump_pcLoad_valid)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(_zz_161_)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(IBusSimplePlugin_fetchPc_samplePcNext)begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      _zz_86_ <= 1'b1;
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        _zz_92_ <= 1'b0;
      end
      if(_zz_90_)begin
        _zz_92_ <= IBusSimplePlugin_iBusRsp_stages_0_output_valid;
      end
      if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
        _zz_94_ <= IBusSimplePlugin_iBusRsp_stages_1_output_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        _zz_94_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
        _zz_96_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        _zz_96_ <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_1_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_2_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_injector_decodeInput_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= IBusSimplePlugin_injector_nextPcCalc_0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_2 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_2 <= IBusSimplePlugin_injector_nextPcCalc_1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_2 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_3 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_3 <= IBusSimplePlugin_injector_nextPcCalc_2;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_3 <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      end
      IBusSimplePlugin_pendingCmd <= IBusSimplePlugin_pendingCmdNext;
      IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_rspJoin_discardCounter - _zz_176_);
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_75_))begin
        IBusSimplePlugin_rspJoin_discardCounter <= IBusSimplePlugin_pendingCmdNext;
      end
      CsrPlugin_mip_MEIP <= externalInterrupt;
      CsrPlugin_mip_MTIP <= timerInterrupt;
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_159_)begin
        CsrPlugin_privilege <= CsrPlugin_targetPrivilege;
        case(CsrPlugin_targetPrivilege)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= 1'b0;
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          end
          default : begin
          end
        endcase
      end
      if(_zz_160_)begin
        case(_zz_164_)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MPIE <= 1'b1;
            CsrPlugin_privilege <= CsrPlugin_mstatus_MPP;
          end
          default : begin
          end
        endcase
      end
      _zz_125_ <= 1'b0;
      if(_zz_154_)begin
        if(_zz_155_)begin
          execute_LightShifterPlugin_isActive <= 1'b1;
          if(execute_LightShifterPlugin_done)begin
            execute_LightShifterPlugin_isActive <= 1'b0;
          end
        end
      end
      if(execute_arbitration_removeIt)begin
        execute_LightShifterPlugin_isActive <= 1'b0;
      end
      _zz_137_ <= _zz_136_;
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= memory_REGFILE_WRITE_DATA;
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(_zz_150_)
        3'b000 : begin
          if(_zz_82_)begin
            _zz_150_ <= (3'b001);
          end
        end
        3'b001 : begin
          _zz_150_ <= (3'b010);
        end
        3'b010 : begin
          _zz_150_ <= (3'b011);
        end
        3'b011 : begin
          if((! decode_arbitration_isStuck))begin
            _zz_150_ <= (3'b100);
          end
        end
        3'b100 : begin
          _zz_150_ <= (3'b000);
        end
        default : begin
        end
      endcase
      case(execute_CsrPlugin_csrAddress)
        12'b001100000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
            CsrPlugin_mstatus_MPIE <= _zz_205_[0];
            CsrPlugin_mstatus_MIE <= _zz_206_[0];
          end
        end
        12'b001101000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mip_MSIP <= _zz_207_[0];
          end
        end
        12'b001100000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mie_MEIE <= _zz_208_[0];
            CsrPlugin_mie_MTIE <= _zz_209_[0];
            CsrPlugin_mie_MSIE <= _zz_210_[0];
          end
        end
        12'b001101000010 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
      _zz_95_ <= IBusSimplePlugin_iBusRsp_stages_1_output_payload;
    end
    if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
      _zz_97_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
      _zz_98_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
      _zz_99_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw;
      _zz_100_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw;
    end
    if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow memory stage stall when read happend");
    end
    if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_INSTRUCTION[5])) && writeBack_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow writeback stage stall when read happend");
    end
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
    if((CsrPlugin_exception || CsrPlugin_interruptJump))begin
      case(CsrPlugin_privilege)
        2'b11 : begin
          CsrPlugin_mepc <= decode_PC;
        end
        default : begin
        end
      endcase
    end
    if(_zz_159_)begin
      case(CsrPlugin_targetPrivilege)
        2'b11 : begin
          CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
        end
        default : begin
        end
      endcase
    end
    if(_zz_154_)begin
      if(_zz_155_)begin
        execute_LightShifterPlugin_amplitudeReg <= (execute_LightShifterPlugin_amplitude - (5'b00001));
      end
    end
    if(_zz_136_)begin
      _zz_138_ <= _zz_38_[11 : 7];
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_18_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_15_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_12_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_10_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_DO_EBREAK <= decode_DO_EBREAK;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_69_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_29_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_8_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_5_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_60_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_28_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_32_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_2_;
    end
    if((_zz_150_ != (3'b000)))begin
      _zz_99_ <= debug_bus_cmd_payload_data;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    DebugPlugin_firstCycle <= 1'b0;
    if(debug_bus_cmd_ready)begin
      DebugPlugin_firstCycle <= 1'b1;
    end
    DebugPlugin_secondCycle <= DebugPlugin_firstCycle;
    DebugPlugin_isPipActive <= ({writeBack_arbitration_isValid,{memory_arbitration_isValid,{execute_arbitration_isValid,decode_arbitration_isValid}}} != (4'b0000));
    DebugPlugin_isPipActive_regNext <= DebugPlugin_isPipActive;
    if(writeBack_arbitration_isValid)begin
      DebugPlugin_busReadDataReg <= _zz_66_;
    end
    _zz_149_ <= debug_bus_cmd_payload_address[2];
    if(_zz_156_)begin
      DebugPlugin_busReadDataReg <= execute_PC;
    end
    DebugPlugin_resetIt_regNext <= DebugPlugin_resetIt;
  end

  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_mainClkReset) begin
    if (toplevel_resetCtrl_mainClkReset) begin
      DebugPlugin_resetIt <= 1'b0;
      DebugPlugin_haltIt <= 1'b0;
      DebugPlugin_stepIt <= 1'b0;
      DebugPlugin_haltedByBreak <= 1'b0;
    end else begin
      if(debug_bus_cmd_valid)begin
        case(_zz_162_)
          6'b000000 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_stepIt <= debug_bus_cmd_payload_data[4];
              if(debug_bus_cmd_payload_data[16])begin
                DebugPlugin_resetIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[24])begin
                DebugPlugin_resetIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[17])begin
                DebugPlugin_haltIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltedByBreak <= 1'b0;
              end
            end
          end
          6'b000001 : begin
          end
          default : begin
          end
        endcase
      end
      if(_zz_156_)begin
        if(_zz_157_)begin
          DebugPlugin_haltIt <= 1'b1;
          DebugPlugin_haltedByBreak <= 1'b1;
        end
      end
      if(_zz_158_)begin
        if(decode_arbitration_isValid)begin
          DebugPlugin_haltIt <= 1'b1;
        end
      end
      if((DebugPlugin_stepIt && ({writeBack_arbitration_redoIt,{memory_arbitration_redoIt,{execute_arbitration_redoIt,decode_arbitration_redoIt}}} != (4'b0000))))begin
        DebugPlugin_haltIt <= 1'b0;
      end
    end
  end

endmodule

module JtagBridge (
      input   io_jtag_tms,
      input   io_jtag_tdi,
      output reg  io_jtag_tdo,
      input   io_jtag_tck,
      output  io_remote_cmd_valid,
      input   io_remote_cmd_ready,
      output  io_remote_cmd_payload_last,
      output [0:0] io_remote_cmd_payload_fragment,
      input   io_remote_rsp_valid,
      output  io_remote_rsp_ready,
      input   io_remote_rsp_payload_error,
      input  [31:0] io_remote_rsp_payload_data,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_mainClkReset);
  wire  flowCCByToggle_1__io_output_valid;
  wire  flowCCByToggle_1__io_output_payload_last;
  wire [0:0] flowCCByToggle_1__io_output_payload_fragment;
  wire  _zz_2_;
  wire  _zz_3_;
  wire [3:0] _zz_4_;
  wire [3:0] _zz_5_;
  wire [3:0] _zz_6_;
  wire  system_cmd_valid;
  wire  system_cmd_payload_last;
  wire [0:0] system_cmd_payload_fragment;
  reg  system_rsp_valid;
  reg  system_rsp_payload_error;
  reg [31:0] system_rsp_payload_data;
  wire `JtagState_defaultEncoding_type jtag_tap_fsm_stateNext;
  reg `JtagState_defaultEncoding_type jtag_tap_fsm_state = `JtagState_defaultEncoding_RESET;
  reg `JtagState_defaultEncoding_type _zz_1_;
  reg [3:0] jtag_tap_instruction;
  reg [3:0] jtag_tap_instructionShift;
  reg  jtag_tap_bypass;
  wire [0:0] jtag_idcodeArea_instructionId;
  wire  jtag_idcodeArea_instructionHit;
  reg [31:0] jtag_idcodeArea_shifter;
  wire [1:0] jtag_writeArea_instructionId;
  wire  jtag_writeArea_instructionHit;
  reg  jtag_writeArea_source_valid;
  wire  jtag_writeArea_source_payload_last;
  wire [0:0] jtag_writeArea_source_payload_fragment;
  wire [1:0] jtag_readArea_instructionId;
  wire  jtag_readArea_instructionHit;
  reg [33:0] jtag_readArea_shifter;
  `ifndef SYNTHESIS
  reg [79:0] jtag_tap_fsm_stateNext_string;
  reg [79:0] jtag_tap_fsm_state_string;
  reg [79:0] _zz_1__string;
  `endif

  assign _zz_2_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_3_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_4_ = {3'd0, jtag_idcodeArea_instructionId};
  assign _zz_5_ = {2'd0, jtag_writeArea_instructionId};
  assign _zz_6_ = {2'd0, jtag_readArea_instructionId};
  FlowCCByToggle flowCCByToggle_1_ ( 
    .io_input_valid(jtag_writeArea_source_valid),
    .io_input_payload_last(jtag_writeArea_source_payload_last),
    .io_input_payload_fragment(jtag_writeArea_source_payload_fragment),
    .io_output_valid(flowCCByToggle_1__io_output_valid),
    .io_output_payload_last(flowCCByToggle_1__io_output_payload_last),
    .io_output_payload_fragment(flowCCByToggle_1__io_output_payload_fragment),
    ._zz_1_(io_jtag_tck),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_mainClkReset(toplevel_resetCtrl_mainClkReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(jtag_tap_fsm_stateNext)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_stateNext_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_stateNext_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_stateNext_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_stateNext_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_stateNext_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_stateNext_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_stateNext_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_stateNext_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_stateNext_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_stateNext_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_stateNext_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_stateNext_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_stateNext_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_stateNext_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_stateNext_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_stateNext_string = "DR_UPDATE ";
      default : jtag_tap_fsm_stateNext_string = "??????????";
    endcase
  end
  always @(*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_state_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_state_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_state_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_state_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_state_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_state_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_state_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_state_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_state_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_state_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_state_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_state_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_state_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_state_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_state_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_state_string = "DR_UPDATE ";
      default : jtag_tap_fsm_state_string = "??????????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `JtagState_defaultEncoding_RESET : _zz_1__string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : _zz_1__string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : _zz_1__string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : _zz_1__string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : _zz_1__string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : _zz_1__string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : _zz_1__string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : _zz_1__string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : _zz_1__string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : _zz_1__string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : _zz_1__string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : _zz_1__string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : _zz_1__string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : _zz_1__string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : _zz_1__string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : _zz_1__string = "DR_UPDATE ";
      default : _zz_1__string = "??????????";
    endcase
  end
  `endif

  assign io_remote_cmd_valid = system_cmd_valid;
  assign io_remote_cmd_payload_last = system_cmd_payload_last;
  assign io_remote_cmd_payload_fragment = system_cmd_payload_fragment;
  assign io_remote_rsp_ready = 1'b1;
  always @ (*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IDLE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_IR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IR_CAPTURE);
      end
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT2 : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_DR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_SELECT : `JtagState_defaultEncoding_DR_CAPTURE);
      end
      `JtagState_defaultEncoding_DR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT2 : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      default : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IDLE);
      end
    endcase
  end

  assign jtag_tap_fsm_stateNext = _zz_1_;
  always @ (*) begin
    io_jtag_tdo = jtag_tap_bypass;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        io_jtag_tdo = jtag_tap_instructionShift[0];
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        io_jtag_tdo = jtag_idcodeArea_shifter[0];
      end
    end
    if(jtag_readArea_instructionHit)begin
      if(_zz_3_)begin
        io_jtag_tdo = jtag_readArea_shifter[0];
      end
    end
  end

  assign jtag_idcodeArea_instructionId = (1'b1);
  assign jtag_idcodeArea_instructionHit = (jtag_tap_instruction == _zz_4_);
  assign jtag_writeArea_instructionId = (2'b10);
  assign jtag_writeArea_instructionHit = (jtag_tap_instruction == _zz_5_);
  always @ (*) begin
    jtag_writeArea_source_valid = 1'b0;
    if(jtag_writeArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT))begin
        jtag_writeArea_source_valid = 1'b1;
      end
    end
  end

  assign jtag_writeArea_source_payload_last = io_jtag_tms;
  assign jtag_writeArea_source_payload_fragment[0] = io_jtag_tdi;
  assign system_cmd_valid = flowCCByToggle_1__io_output_valid;
  assign system_cmd_payload_last = flowCCByToggle_1__io_output_payload_last;
  assign system_cmd_payload_fragment = flowCCByToggle_1__io_output_payload_fragment;
  assign jtag_readArea_instructionId = (2'b11);
  assign jtag_readArea_instructionHit = (jtag_tap_instruction == _zz_6_);
  always @ (posedge toplevel_io_mainClk) begin
    if(io_remote_cmd_valid)begin
      system_rsp_valid <= 1'b0;
    end
    if((io_remote_rsp_valid && io_remote_rsp_ready))begin
      system_rsp_valid <= 1'b1;
      system_rsp_payload_error <= io_remote_rsp_payload_error;
      system_rsp_payload_data <= io_remote_rsp_payload_data;
    end
  end

  always @ (posedge io_jtag_tck) begin
    jtag_tap_fsm_state <= jtag_tap_fsm_stateNext;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        jtag_tap_instructionShift <= jtag_tap_instruction;
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_instructionShift <= ({io_jtag_tdi,jtag_tap_instructionShift} >>> 1);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        jtag_tap_instruction <= jtag_tap_instructionShift;
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        jtag_tap_bypass <= io_jtag_tdi;
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_idcodeArea_shifter <= ({io_jtag_tdi,jtag_idcodeArea_shifter} >>> 1);
      end
    end
    if((jtag_tap_fsm_state == `JtagState_defaultEncoding_RESET))begin
      jtag_idcodeArea_shifter <= (32'b00010000000000000001111111111111);
      jtag_tap_instruction <= {3'd0, jtag_idcodeArea_instructionId};
    end
    if(jtag_readArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_CAPTURE))begin
        jtag_readArea_shifter <= {{system_rsp_payload_data,system_rsp_payload_error},system_rsp_valid};
      end
      if(_zz_3_)begin
        jtag_readArea_shifter <= ({io_jtag_tdi,jtag_readArea_shifter} >>> 1);
      end
    end
  end

endmodule

module SystemDebugger (
      input   io_remote_cmd_valid,
      output  io_remote_cmd_ready,
      input   io_remote_cmd_payload_last,
      input  [0:0] io_remote_cmd_payload_fragment,
      output  io_remote_rsp_valid,
      input   io_remote_rsp_ready,
      output  io_remote_rsp_payload_error,
      output [31:0] io_remote_rsp_payload_data,
      output  io_mem_cmd_valid,
      input   io_mem_cmd_ready,
      output [31:0] io_mem_cmd_payload_address,
      output [31:0] io_mem_cmd_payload_data,
      output  io_mem_cmd_payload_wr,
      output [1:0] io_mem_cmd_payload_size,
      input   io_mem_rsp_valid,
      input  [31:0] io_mem_rsp_payload,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_mainClkReset);
  wire  _zz_2_;
  wire [0:0] _zz_3_;
  reg [66:0] dispatcher_dataShifter;
  reg  dispatcher_dataLoaded;
  reg [7:0] dispatcher_headerShifter;
  wire [7:0] dispatcher_header;
  reg  dispatcher_headerLoaded;
  reg [2:0] dispatcher_counter;
  wire [66:0] _zz_1_;
  assign _zz_2_ = (dispatcher_headerLoaded == 1'b0);
  assign _zz_3_ = _zz_1_[64 : 64];
  assign dispatcher_header = dispatcher_headerShifter[7 : 0];
  assign io_remote_cmd_ready = (! dispatcher_dataLoaded);
  assign _zz_1_ = dispatcher_dataShifter[66 : 0];
  assign io_mem_cmd_payload_address = _zz_1_[31 : 0];
  assign io_mem_cmd_payload_data = _zz_1_[63 : 32];
  assign io_mem_cmd_payload_wr = _zz_3_[0];
  assign io_mem_cmd_payload_size = _zz_1_[66 : 65];
  assign io_mem_cmd_valid = (dispatcher_dataLoaded && (dispatcher_header == (8'b00000000)));
  assign io_remote_rsp_valid = io_mem_rsp_valid;
  assign io_remote_rsp_payload_error = 1'b0;
  assign io_remote_rsp_payload_data = io_mem_rsp_payload;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_mainClkReset) begin
    if (toplevel_resetCtrl_mainClkReset) begin
      dispatcher_dataLoaded <= 1'b0;
      dispatcher_headerLoaded <= 1'b0;
      dispatcher_counter <= (3'b000);
    end else begin
      if(io_remote_cmd_valid)begin
        if(_zz_2_)begin
          dispatcher_counter <= (dispatcher_counter + (3'b001));
          if((dispatcher_counter == (3'b111)))begin
            dispatcher_headerLoaded <= 1'b1;
          end
        end
        if(io_remote_cmd_payload_last)begin
          dispatcher_headerLoaded <= 1'b1;
          dispatcher_dataLoaded <= 1'b1;
          dispatcher_counter <= (3'b000);
        end
      end
      if((io_mem_cmd_valid && io_mem_cmd_ready))begin
        dispatcher_headerLoaded <= 1'b0;
        dispatcher_dataLoaded <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(io_remote_cmd_valid)begin
      if(_zz_2_)begin
        dispatcher_headerShifter <= ({io_remote_cmd_payload_fragment,dispatcher_headerShifter} >>> 1);
      end else begin
        dispatcher_dataShifter <= ({io_remote_cmd_payload_fragment,dispatcher_dataShifter} >>> 1);
      end
    end
  end

endmodule

module MuraxPipelinedMemoryBusRam (
      input   io_bus_cmd_valid,
      output  io_bus_cmd_ready,
      input   io_bus_cmd_payload_write,
      input  [31:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_0_data,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [31:0] _zz_4_;
  wire [11:0] _zz_5_;
  reg  _zz_1_;
  wire [29:0] _zz_2_;
  wire [31:0] _zz_3_;
  reg [7:0] ram_symbol0 [0:3071];
  reg [7:0] ram_symbol1 [0:3071];
  reg [7:0] ram_symbol2 [0:3071];
  reg [7:0] ram_symbol3 [0:3071];
  reg [7:0] _zz_6_;
  reg [7:0] _zz_7_;
  reg [7:0] _zz_8_;
  reg [7:0] _zz_9_;
  assign _zz_5_ = _zz_2_[11:0];
  initial begin
    $readmemb("MuraxArduino.v_toplevel_system_ram_ram_symbol0.bin",ram_symbol0);
    $readmemb("MuraxArduino.v_toplevel_system_ram_ram_symbol1.bin",ram_symbol1);
    $readmemb("MuraxArduino.v_toplevel_system_ram_ram_symbol2.bin",ram_symbol2);
    $readmemb("MuraxArduino.v_toplevel_system_ram_ram_symbol3.bin",ram_symbol3);
  end
  always @ (*) begin
    _zz_4_ = {_zz_9_, _zz_8_, _zz_7_, _zz_6_};
  end
  always @ (posedge toplevel_io_mainClk) begin
    if(io_bus_cmd_payload_mask[0] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol0[_zz_5_] <= _zz_3_[7 : 0];
    end
    if(io_bus_cmd_payload_mask[1] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol1[_zz_5_] <= _zz_3_[15 : 8];
    end
    if(io_bus_cmd_payload_mask[2] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol2[_zz_5_] <= _zz_3_[23 : 16];
    end
    if(io_bus_cmd_payload_mask[3] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol3[_zz_5_] <= _zz_3_[31 : 24];
    end
    if(io_bus_cmd_valid) begin
      _zz_6_ <= ram_symbol0[_zz_5_];
      _zz_7_ <= ram_symbol1[_zz_5_];
      _zz_8_ <= ram_symbol2[_zz_5_];
      _zz_9_ <= ram_symbol3[_zz_5_];
    end
  end

  assign io_bus_rsp_valid = _zz_1_;
  assign _zz_2_ = (io_bus_cmd_payload_address >>> 2);
  assign _zz_3_ = io_bus_cmd_payload_data;
  assign io_bus_rsp_0_data = _zz_4_;
  assign io_bus_cmd_ready = 1'b1;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      _zz_1_ <= 1'b0;
    end else begin
      _zz_1_ <= ((io_bus_cmd_valid && io_bus_cmd_ready) && (! io_bus_cmd_payload_write));
    end
  end

endmodule

module MuraxPipelinedMemoryBusSram (
      input   io_bus_cmd_valid,
      output reg  io_bus_cmd_ready,
      input   io_bus_cmd_payload_write,
      input  [31:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_1_data,
      output [17:0] io_sram_addr,
      input  [15:0] io_sram_dat_read,
      output [15:0] io_sram_dat_write,
      output  io_sram_dat_writeEnable,
      output  io_sram_cs,
      output  io_sram_we,
      output  io_sram_oe,
      output  io_sram_lb,
      output  io_sram_ub,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire  _zz_8_;
  wire  _zz_9_;
  reg  we;
  reg  oe;
  reg  lb;
  reg  ub;
  reg [1:0] state;
  reg [15:0] datOut;
  reg [17:0] addr;
  reg  _zz_1_;
  reg [31:0] rspData;
  assign _zz_2_ = (state == (2'b00));
  assign _zz_3_ = (state == (2'b01));
  assign _zz_4_ = (state == (2'b10));
  assign _zz_5_ = (state == (2'b11));
  assign _zz_6_ = (state == (2'b00));
  assign _zz_7_ = (state == (2'b01));
  assign _zz_8_ = (state == (2'b10));
  assign _zz_9_ = (state == (2'b11));
  assign io_sram_we = (! we);
  assign io_sram_oe = (! oe);
  assign io_sram_lb = (! lb);
  assign io_sram_ub = (! ub);
  assign io_sram_dat_write = datOut;
  assign io_sram_addr = addr;
  assign io_sram_cs = (! io_bus_cmd_valid);
  assign io_bus_rsp_valid = _zz_1_;
  assign io_bus_rsp_1_data = rspData;
  assign io_sram_dat_writeEnable = we;
  always @ (*) begin
    io_bus_cmd_ready = 1'b0;
    if(io_bus_cmd_valid)begin
      if(io_bus_cmd_payload_write)begin
        if(! _zz_2_) begin
          if(! _zz_3_) begin
            if(! _zz_4_) begin
              if(_zz_5_)begin
                io_bus_cmd_ready = 1'b1;
              end
            end
          end
        end
      end else begin
        if(! _zz_6_) begin
          if(! _zz_7_) begin
            if(! _zz_8_) begin
              if(_zz_9_)begin
                io_bus_cmd_ready = 1'b1;
              end
            end
          end
        end
      end
    end
  end

  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      state <= (2'b00);
      _zz_1_ <= 1'b0;
    end else begin
      _zz_1_ <= (io_bus_cmd_ready && (! io_bus_cmd_payload_write));
      if(io_bus_cmd_valid)begin
        if(io_bus_cmd_payload_write)begin
          if(_zz_2_)begin
            state <= (2'b01);
          end else begin
            if(_zz_3_)begin
              state <= (2'b10);
            end else begin
              if(_zz_4_)begin
                state <= (2'b11);
              end else begin
                if(_zz_5_)begin
                  state <= (2'b00);
                end
              end
            end
          end
        end else begin
          if(_zz_6_)begin
            state <= (2'b01);
          end else begin
            if(_zz_7_)begin
              state <= (2'b10);
            end else begin
              if(_zz_8_)begin
                state <= (2'b11);
              end else begin
                if(_zz_9_)begin
                  state <= (2'b00);
                end
              end
            end
          end
        end
      end
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    we <= 1'b0;
    oe <= 1'b0;
    if(io_bus_cmd_valid)begin
      if(io_bus_cmd_payload_write)begin
        if(_zz_2_)begin
          addr <= {io_bus_cmd_payload_address[18 : 2],(1'b0)};
          we <= 1'b1;
          datOut <= io_bus_cmd_payload_data[15 : 0];
          lb <= io_bus_cmd_payload_mask[0];
          ub <= io_bus_cmd_payload_mask[1];
        end else begin
          if(! _zz_3_) begin
            if(_zz_4_)begin
              addr <= {io_bus_cmd_payload_address[18 : 2],(1'b1)};
              we <= 1'b1;
              datOut <= io_bus_cmd_payload_data[31 : 16];
              lb <= io_bus_cmd_payload_mask[2];
              ub <= io_bus_cmd_payload_mask[3];
            end
          end
        end
      end else begin
        lb <= 1'b1;
        ub <= 1'b1;
        if(_zz_6_)begin
          oe <= 1'b1;
          addr <= {io_bus_cmd_payload_address[18 : 2],(1'b0)};
        end else begin
          if(_zz_7_)begin
            rspData[15 : 0] <= io_sram_dat_read;
          end else begin
            if(_zz_8_)begin
              addr <= {io_bus_cmd_payload_address[18 : 2],(1'b1)};
              oe <= 1'b1;
            end else begin
              if(_zz_9_)begin
                rspData[31 : 16] <= io_sram_dat_read;
              end
            end
          end
        end
      end
    end
  end

endmodule

module PipelinedMemoryBusToApbBridge (
      input   io_pipelinedMemoryBus_cmd_valid,
      output  io_pipelinedMemoryBus_cmd_ready,
      input   io_pipelinedMemoryBus_cmd_payload_write,
      input  [31:0] io_pipelinedMemoryBus_cmd_payload_address,
      input  [31:0] io_pipelinedMemoryBus_cmd_payload_data,
      input  [3:0] io_pipelinedMemoryBus_cmd_payload_mask,
      output  io_pipelinedMemoryBus_rsp_valid,
      output [31:0] io_pipelinedMemoryBus_rsp_2_data,
      output [19:0] io_apb_PADDR,
      output [0:0] io_apb_PSEL,
      output  io_apb_PENABLE,
      input   io_apb_PREADY,
      output  io_apb_PWRITE,
      output [31:0] io_apb_PWDATA,
      input  [31:0] io_apb_PRDATA,
      input   io_apb_PSLVERROR,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_1_;
  wire  _zz_2_;
  wire  pipelinedMemoryBusStage_cmd_valid;
  reg  pipelinedMemoryBusStage_cmd_ready;
  wire  pipelinedMemoryBusStage_cmd_payload_write;
  wire [31:0] pipelinedMemoryBusStage_cmd_payload_address;
  wire [31:0] pipelinedMemoryBusStage_cmd_payload_data;
  wire [3:0] pipelinedMemoryBusStage_cmd_payload_mask;
  reg  pipelinedMemoryBusStage_rsp_valid;
  wire [31:0] pipelinedMemoryBusStage_rsp_payload_data;
  wire  io_pipelinedMemoryBus_cmd_halfPipe_valid;
  wire  io_pipelinedMemoryBus_cmd_halfPipe_ready;
  wire  io_pipelinedMemoryBus_cmd_halfPipe_payload_write;
  wire [31:0] io_pipelinedMemoryBus_cmd_halfPipe_payload_address;
  wire [31:0] io_pipelinedMemoryBus_cmd_halfPipe_payload_data;
  wire [3:0] io_pipelinedMemoryBus_cmd_halfPipe_payload_mask;
  reg  io_pipelinedMemoryBus_cmd_halfPipe_regs_valid;
  reg  io_pipelinedMemoryBus_cmd_halfPipe_regs_ready;
  reg  io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_write;
  reg [31:0] io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_address;
  reg [31:0] io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_data;
  reg [3:0] io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_mask;
  reg  pipelinedMemoryBusStage_rsp_m2sPipe_valid;
  reg [31:0] pipelinedMemoryBusStage_rsp_m2sPipe_payload_data;
  reg  state;
  assign _zz_1_ = (! state);
  assign _zz_2_ = (! io_pipelinedMemoryBus_cmd_halfPipe_regs_valid);
  assign io_pipelinedMemoryBus_cmd_halfPipe_valid = io_pipelinedMemoryBus_cmd_halfPipe_regs_valid;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_write = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_write;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_address = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_address;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_data = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_data;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_mask = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_mask;
  assign io_pipelinedMemoryBus_cmd_ready = io_pipelinedMemoryBus_cmd_halfPipe_regs_ready;
  assign pipelinedMemoryBusStage_cmd_valid = io_pipelinedMemoryBus_cmd_halfPipe_valid;
  assign io_pipelinedMemoryBus_cmd_halfPipe_ready = pipelinedMemoryBusStage_cmd_ready;
  assign pipelinedMemoryBusStage_cmd_payload_write = io_pipelinedMemoryBus_cmd_halfPipe_payload_write;
  assign pipelinedMemoryBusStage_cmd_payload_address = io_pipelinedMemoryBus_cmd_halfPipe_payload_address;
  assign pipelinedMemoryBusStage_cmd_payload_data = io_pipelinedMemoryBus_cmd_halfPipe_payload_data;
  assign pipelinedMemoryBusStage_cmd_payload_mask = io_pipelinedMemoryBus_cmd_halfPipe_payload_mask;
  assign io_pipelinedMemoryBus_rsp_valid = pipelinedMemoryBusStage_rsp_m2sPipe_valid;
  assign io_pipelinedMemoryBus_rsp_2_data = pipelinedMemoryBusStage_rsp_m2sPipe_payload_data;
  always @ (*) begin
    pipelinedMemoryBusStage_cmd_ready = 1'b0;
    pipelinedMemoryBusStage_rsp_valid = 1'b0;
    if(! _zz_1_) begin
      if(io_apb_PREADY)begin
        pipelinedMemoryBusStage_rsp_valid = (! pipelinedMemoryBusStage_cmd_payload_write);
        pipelinedMemoryBusStage_cmd_ready = 1'b1;
      end
    end
  end

  assign io_apb_PSEL[0] = pipelinedMemoryBusStage_cmd_valid;
  assign io_apb_PENABLE = state;
  assign io_apb_PWRITE = pipelinedMemoryBusStage_cmd_payload_write;
  assign io_apb_PADDR = pipelinedMemoryBusStage_cmd_payload_address[19:0];
  assign io_apb_PWDATA = pipelinedMemoryBusStage_cmd_payload_data;
  assign pipelinedMemoryBusStage_rsp_payload_data = io_apb_PRDATA;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      io_pipelinedMemoryBus_cmd_halfPipe_regs_valid <= 1'b0;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_ready <= 1'b1;
      pipelinedMemoryBusStage_rsp_m2sPipe_valid <= 1'b0;
      state <= 1'b0;
    end else begin
      if(_zz_2_)begin
        io_pipelinedMemoryBus_cmd_halfPipe_regs_valid <= io_pipelinedMemoryBus_cmd_valid;
        io_pipelinedMemoryBus_cmd_halfPipe_regs_ready <= (! io_pipelinedMemoryBus_cmd_valid);
      end else begin
        io_pipelinedMemoryBus_cmd_halfPipe_regs_valid <= (! io_pipelinedMemoryBus_cmd_halfPipe_ready);
        io_pipelinedMemoryBus_cmd_halfPipe_regs_ready <= io_pipelinedMemoryBus_cmd_halfPipe_ready;
      end
      pipelinedMemoryBusStage_rsp_m2sPipe_valid <= pipelinedMemoryBusStage_rsp_valid;
      if(_zz_1_)begin
        state <= pipelinedMemoryBusStage_cmd_valid;
      end else begin
        if(io_apb_PREADY)begin
          state <= 1'b0;
        end
      end
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    if(_zz_2_)begin
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_write <= io_pipelinedMemoryBus_cmd_payload_write;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_address <= io_pipelinedMemoryBus_cmd_payload_address;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_data <= io_pipelinedMemoryBus_cmd_payload_data;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_mask <= io_pipelinedMemoryBus_cmd_payload_mask;
    end
    if(pipelinedMemoryBusStage_rsp_valid)begin
      pipelinedMemoryBusStage_rsp_m2sPipe_payload_data <= pipelinedMemoryBusStage_rsp_payload_data;
    end
  end

endmodule

module Apb3Gpio (
      input  [3:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input  [31:0] io_gpio_read,
      output [31:0] io_gpio_write,
      output [31:0] io_gpio_writeEnable,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  ctrl_askWrite;
  wire  ctrl_askRead;
  wire  ctrl_doWrite;
  wire  ctrl_doRead;
  reg [31:0] _zz_1_;
  reg [31:0] _zz_2_;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      4'b0000 : begin
        io_apb_PRDATA[31 : 0] = io_gpio_read;
      end
      4'b0100 : begin
        io_apb_PRDATA[31 : 0] = _zz_1_;
      end
      4'b1000 : begin
        io_apb_PRDATA[31 : 0] = _zz_2_;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign ctrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign ctrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign ctrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign ctrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_gpio_write = _zz_1_;
  assign io_gpio_writeEnable = _zz_2_;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      _zz_2_ <= (32'b00000000000000000000000000000000);
    end else begin
      case(io_apb_PADDR)
        4'b0000 : begin
        end
        4'b0100 : begin
        end
        4'b1000 : begin
          if(ctrl_doWrite)begin
            _zz_2_ <= io_apb_PWDATA[31 : 0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      4'b0000 : begin
      end
      4'b0100 : begin
        if(ctrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[31 : 0];
        end
      end
      4'b1000 : begin
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3UartCtrl (
      input  [3:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_uart_txd,
      input   io_uart_rxd,
      output  io_interrupt,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_3_;
  reg  _zz_4_;
  wire  _zz_5_;
  wire  uartCtrl_1__io_write_ready;
  wire  uartCtrl_1__io_read_valid;
  wire [7:0] uartCtrl_1__io_read_payload;
  wire  uartCtrl_1__io_uart_txd;
  wire  bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  wire  bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
  wire [7:0] bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload;
  wire [4:0] bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy;
  wire [4:0] bridge_write_streamUnbuffered_queueWithOccupancy_io_availability;
  wire  uartCtrl_1__io_read_queueWithOccupancy_io_push_ready;
  wire  uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid;
  wire [7:0] uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload;
  wire [4:0] uartCtrl_1__io_read_queueWithOccupancy_io_occupancy;
  wire [4:0] uartCtrl_1__io_read_queueWithOccupancy_io_availability;
  wire [0:0] _zz_6_;
  wire [0:0] _zz_7_;
  wire [4:0] _zz_8_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  wire [2:0] bridge_uartConfigReg_frame_dataLength;
  wire `UartStopType_defaultEncoding_type bridge_uartConfigReg_frame_stop;
  wire `UartParityType_defaultEncoding_type bridge_uartConfigReg_frame_parity;
  reg [19:0] bridge_uartConfigReg_clockDivider;
  reg  _zz_1_;
  wire  bridge_write_streamUnbuffered_valid;
  wire  bridge_write_streamUnbuffered_ready;
  wire [7:0] bridge_write_streamUnbuffered_payload;
  reg  bridge_interruptCtrl_writeIntEnable;
  reg  bridge_interruptCtrl_readIntEnable;
  wire  bridge_interruptCtrl_readInt;
  wire  bridge_interruptCtrl_writeInt;
  wire  bridge_interruptCtrl_interrupt;
  wire [7:0] _zz_2_;
  `ifndef SYNTHESIS
  reg [23:0] bridge_uartConfigReg_frame_stop_string;
  reg [31:0] bridge_uartConfigReg_frame_parity_string;
  `endif

  function [19:0] zz_bridge_uartConfigReg_clockDivider(input dummy);
    begin
      zz_bridge_uartConfigReg_clockDivider = (20'b00000000000000000000);
      zz_bridge_uartConfigReg_clockDivider = (20'b00000000000001010101);
    end
  endfunction
  wire [19:0] _zz_9_;
  assign _zz_6_ = io_apb_PWDATA[0 : 0];
  assign _zz_7_ = io_apb_PWDATA[1 : 1];
  assign _zz_8_ = ((5'b10000) - bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy);
  UartCtrl uartCtrl_1_ ( 
    .io_config_frame_dataLength(bridge_uartConfigReg_frame_dataLength),
    .io_config_frame_stop(bridge_uartConfigReg_frame_stop),
    .io_config_frame_parity(bridge_uartConfigReg_frame_parity),
    .io_config_clockDivider(bridge_uartConfigReg_clockDivider),
    .io_write_valid(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid),
    .io_write_ready(uartCtrl_1__io_write_ready),
    .io_write_payload(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload),
    .io_read_valid(uartCtrl_1__io_read_valid),
    .io_read_payload(uartCtrl_1__io_read_payload),
    .io_uart_txd(uartCtrl_1__io_uart_txd),
    .io_uart_rxd(io_uart_rxd),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  StreamFifo bridge_write_streamUnbuffered_queueWithOccupancy ( 
    .io_push_valid(bridge_write_streamUnbuffered_valid),
    .io_push_ready(bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready),
    .io_push_payload(bridge_write_streamUnbuffered_payload),
    .io_pop_valid(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(uartCtrl_1__io_write_ready),
    .io_pop_payload(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload),
    .io_flush(_zz_3_),
    .io_occupancy(bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy),
    .io_availability(bridge_write_streamUnbuffered_queueWithOccupancy_io_availability),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  StreamFifo uartCtrl_1__io_read_queueWithOccupancy ( 
    .io_push_valid(uartCtrl_1__io_read_valid),
    .io_push_ready(uartCtrl_1__io_read_queueWithOccupancy_io_push_ready),
    .io_push_payload(uartCtrl_1__io_read_payload),
    .io_pop_valid(uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(_zz_4_),
    .io_pop_payload(uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload),
    .io_flush(_zz_5_),
    .io_occupancy(uartCtrl_1__io_read_queueWithOccupancy_io_occupancy),
    .io_availability(uartCtrl_1__io_read_queueWithOccupancy_io_availability),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(bridge_uartConfigReg_frame_stop)
      `UartStopType_defaultEncoding_ONE : bridge_uartConfigReg_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : bridge_uartConfigReg_frame_stop_string = "TWO";
      default : bridge_uartConfigReg_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(bridge_uartConfigReg_frame_parity)
      `UartParityType_defaultEncoding_NONE : bridge_uartConfigReg_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : bridge_uartConfigReg_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : bridge_uartConfigReg_frame_parity_string = "ODD ";
      default : bridge_uartConfigReg_frame_parity_string = "????";
    endcase
  end
  `endif

  assign io_uart_txd = uartCtrl_1__io_uart_txd;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_1_ = 1'b0;
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      4'b0000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ = 1'b1;
        end
        if(busCtrl_doRead)begin
          _zz_4_ = 1'b1;
        end
        io_apb_PRDATA[16 : 16] = uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid;
        io_apb_PRDATA[7 : 0] = uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload;
      end
      4'b0100 : begin
        io_apb_PRDATA[20 : 16] = _zz_8_;
        io_apb_PRDATA[28 : 24] = uartCtrl_1__io_read_queueWithOccupancy_io_occupancy;
        io_apb_PRDATA[0 : 0] = bridge_interruptCtrl_writeIntEnable;
        io_apb_PRDATA[1 : 1] = bridge_interruptCtrl_readIntEnable;
        io_apb_PRDATA[8 : 8] = bridge_interruptCtrl_writeInt;
        io_apb_PRDATA[9 : 9] = bridge_interruptCtrl_readInt;
      end
      default : begin
      end
    endcase
  end

  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign _zz_9_ = zz_bridge_uartConfigReg_clockDivider(1'b0);
  always @ (*) bridge_uartConfigReg_clockDivider = _zz_9_;
  assign bridge_uartConfigReg_frame_dataLength = (3'b111);
  assign bridge_uartConfigReg_frame_parity = `UartParityType_defaultEncoding_NONE;
  assign bridge_uartConfigReg_frame_stop = `UartStopType_defaultEncoding_ONE;
  assign bridge_write_streamUnbuffered_valid = _zz_1_;
  assign bridge_write_streamUnbuffered_payload = _zz_2_;
  assign bridge_write_streamUnbuffered_ready = bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  assign bridge_interruptCtrl_readInt = (bridge_interruptCtrl_readIntEnable && uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid);
  assign bridge_interruptCtrl_writeInt = (bridge_interruptCtrl_writeIntEnable && (! bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid));
  assign bridge_interruptCtrl_interrupt = (bridge_interruptCtrl_readInt || bridge_interruptCtrl_writeInt);
  assign io_interrupt = bridge_interruptCtrl_interrupt;
  assign _zz_2_ = io_apb_PWDATA[7 : 0];
  assign _zz_3_ = 1'b0;
  assign _zz_5_ = 1'b0;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      bridge_interruptCtrl_writeIntEnable <= 1'b0;
      bridge_interruptCtrl_readIntEnable <= 1'b0;
    end else begin
      case(io_apb_PADDR)
        4'b0000 : begin
        end
        4'b0100 : begin
          if(busCtrl_doWrite)begin
            bridge_interruptCtrl_writeIntEnable <= _zz_6_[0];
            bridge_interruptCtrl_readIntEnable <= _zz_7_[0];
          end
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module Apb3PinInterruptCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input  [1:0] io_pinInterrupt_pins,
      output  io_interrupt,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg [1:0] _zz_4_;
  reg [1:0] _zz_5_;
  wire [1:0] pinInterruptCtrl_1__io_interrupt;
  wire [1:0] interruptCtrl_2__io_pendings;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [1:0] _zz_1_;
  reg [1:0] _zz_2_;
  reg [1:0] _zz_3_;
  PinInterruptCtrl pinInterruptCtrl_1_ ( 
    .io_pinInterrupt_pins(io_pinInterrupt_pins),
    .io_rising(_zz_2_),
    .io_falling(_zz_3_),
    .io_interrupt(pinInterruptCtrl_1__io_interrupt),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  InterruptCtrl interruptCtrl_2_ ( 
    .io_inputs(_zz_4_),
    .io_clears(_zz_5_),
    .io_masks(_zz_1_),
    .io_pendings(interruptCtrl_2__io_pendings),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_5_ = (2'b00);
    case(io_apb_PADDR)
      8'b00010000 : begin
        if(busCtrl_doWrite)begin
          _zz_5_ = io_apb_PWDATA[1 : 0];
        end
        io_apb_PRDATA[1 : 0] = interruptCtrl_2__io_pendings;
      end
      8'b00010100 : begin
        io_apb_PRDATA[1 : 0] = _zz_1_;
      end
      8'b00000000 : begin
        io_apb_PRDATA[1 : 0] = _zz_2_;
      end
      8'b00000100 : begin
        io_apb_PRDATA[1 : 0] = _zz_3_;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    _zz_4_[0] = pinInterruptCtrl_1__io_interrupt[0];
    _zz_4_[1] = pinInterruptCtrl_1__io_interrupt[1];
  end

  assign io_interrupt = (interruptCtrl_2__io_pendings != (2'b00));
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      _zz_1_ <= (2'b00);
    end else begin
      case(io_apb_PADDR)
        8'b00010000 : begin
        end
        8'b00010100 : begin
          if(busCtrl_doWrite)begin
            _zz_1_ <= io_apb_PWDATA[1 : 0];
          end
        end
        8'b00000000 : begin
        end
        8'b00000100 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= io_apb_PWDATA[1 : 0];
        end
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= io_apb_PWDATA[1 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module MuraxApb3Timer (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output  io_interrupt,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_10_;
  wire  _zz_11_;
  wire  _zz_12_;
  wire  _zz_13_;
  reg [1:0] _zz_14_;
  reg [1:0] _zz_15_;
  wire  prescaler_1__io_overflow;
  wire  timerA_io_full;
  wire [15:0] timerA_io_value;
  wire  timerB_io_full;
  wire [15:0] timerB_io_value;
  wire [1:0] interruptCtrl_2__io_pendings;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [15:0] _zz_1_;
  reg  _zz_2_;
  reg [1:0] timerABridge_ticksEnable;
  reg [0:0] timerABridge_clearsEnable;
  reg  timerABridge_busClearing;
  reg [15:0] _zz_3_;
  reg  _zz_4_;
  reg  _zz_5_;
  reg [1:0] timerBBridge_ticksEnable;
  reg [0:0] timerBBridge_clearsEnable;
  reg  timerBBridge_busClearing;
  reg [15:0] _zz_6_;
  reg  _zz_7_;
  reg  _zz_8_;
  reg [1:0] _zz_9_;
  Prescaler prescaler_1_ ( 
    .io_clear(_zz_2_),
    .io_limit(_zz_1_),
    .io_overflow(prescaler_1__io_overflow),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  Timer timerA ( 
    .io_tick(_zz_10_),
    .io_clear(_zz_11_),
    .io_limit(_zz_3_),
    .io_full(timerA_io_full),
    .io_value(timerA_io_value),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  Timer timerB ( 
    .io_tick(_zz_12_),
    .io_clear(_zz_13_),
    .io_limit(_zz_6_),
    .io_full(timerB_io_full),
    .io_value(timerB_io_value),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  InterruptCtrl interruptCtrl_2_ ( 
    .io_inputs(_zz_14_),
    .io_clears(_zz_15_),
    .io_masks(_zz_9_),
    .io_pendings(interruptCtrl_2__io_pendings),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_2_ = 1'b0;
    _zz_4_ = 1'b0;
    _zz_5_ = 1'b0;
    _zz_7_ = 1'b0;
    _zz_8_ = 1'b0;
    _zz_15_ = (2'b00);
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = _zz_1_;
      end
      8'b01000000 : begin
        io_apb_PRDATA[1 : 0] = timerABridge_ticksEnable;
        io_apb_PRDATA[16 : 16] = timerABridge_clearsEnable;
      end
      8'b01000100 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = _zz_3_;
      end
      8'b01001000 : begin
        if(busCtrl_doWrite)begin
          _zz_5_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = timerA_io_value;
      end
      8'b01010000 : begin
        io_apb_PRDATA[1 : 0] = timerBBridge_ticksEnable;
        io_apb_PRDATA[16 : 16] = timerBBridge_clearsEnable;
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          _zz_7_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = _zz_6_;
      end
      8'b01011000 : begin
        if(busCtrl_doWrite)begin
          _zz_8_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = timerB_io_value;
      end
      8'b00010000 : begin
        if(busCtrl_doWrite)begin
          _zz_15_ = io_apb_PWDATA[1 : 0];
        end
        io_apb_PRDATA[1 : 0] = interruptCtrl_2__io_pendings;
      end
      8'b00010100 : begin
        io_apb_PRDATA[1 : 0] = _zz_9_;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    timerABridge_busClearing = 1'b0;
    if(_zz_4_)begin
      timerABridge_busClearing = 1'b1;
    end
    if(_zz_5_)begin
      timerABridge_busClearing = 1'b1;
    end
  end

  assign _zz_11_ = (((timerABridge_clearsEnable & timerA_io_full) != (1'b0)) || timerABridge_busClearing);
  assign _zz_10_ = ((timerABridge_ticksEnable & {prescaler_1__io_overflow,1'b1}) != (2'b00));
  always @ (*) begin
    timerBBridge_busClearing = 1'b0;
    if(_zz_7_)begin
      timerBBridge_busClearing = 1'b1;
    end
    if(_zz_8_)begin
      timerBBridge_busClearing = 1'b1;
    end
  end

  assign _zz_13_ = (((timerBBridge_clearsEnable & timerB_io_full) != (1'b0)) || timerBBridge_busClearing);
  assign _zz_12_ = ((timerBBridge_ticksEnable & {prescaler_1__io_overflow,1'b1}) != (2'b00));
  always @ (*) begin
    _zz_14_[0] = timerA_io_full;
    _zz_14_[1] = timerB_io_full;
  end

  assign io_interrupt = (interruptCtrl_2__io_pendings != (2'b00));
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      timerABridge_ticksEnable <= (2'b00);
      timerABridge_clearsEnable <= (1'b0);
      timerBBridge_ticksEnable <= (2'b00);
      timerBBridge_clearsEnable <= (1'b0);
      _zz_9_ <= (2'b00);
    end else begin
      case(io_apb_PADDR)
        8'b00000000 : begin
        end
        8'b01000000 : begin
          if(busCtrl_doWrite)begin
            timerABridge_ticksEnable <= io_apb_PWDATA[1 : 0];
            timerABridge_clearsEnable <= io_apb_PWDATA[16 : 16];
          end
        end
        8'b01000100 : begin
        end
        8'b01001000 : begin
        end
        8'b01010000 : begin
          if(busCtrl_doWrite)begin
            timerBBridge_ticksEnable <= io_apb_PWDATA[1 : 0];
            timerBBridge_clearsEnable <= io_apb_PWDATA[16 : 16];
          end
        end
        8'b01010100 : begin
        end
        8'b01011000 : begin
        end
        8'b00010000 : begin
        end
        8'b00010100 : begin
          if(busCtrl_doWrite)begin
            _zz_9_ <= io_apb_PWDATA[1 : 0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          _zz_6_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3PwmCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output [2:0] io_pwm_pins,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [2:0] pwmCtrl_1__io_pwm_pins;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [7:0] _zz_1_;
  reg [7:0] _zz_2_;
  reg [7:0] _zz_3_;
  PwmCtrl pwmCtrl_1_ ( 
    .io_pwm_pins(pwmCtrl_1__io_pwm_pins),
    .io_duty_0(_zz_1_),
    .io_duty_1(_zz_2_),
    .io_duty_2(_zz_3_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  assign io_apb_PRDATA = (32'b00000000000000000000000000000000);
  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_pwm_pins = pwmCtrl_1__io_pwm_pins;
  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[7 : 0];
        end
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= io_apb_PWDATA[7 : 0];
        end
      end
      8'b00001000 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= io_apb_PWDATA[7 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3ServoCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output  io_servo_pin,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  servoCtrl_1__io_servo_pin;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [11:0] _zz_1_;
  ServoCtrl servoCtrl_1_ ( 
    .io_servo_pin(servoCtrl_1__io_servo_pin),
    .io_pulseMicros(_zz_1_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[11 : 0] = _zz_1_;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_servo_pin = servoCtrl_1__io_servo_pin;
  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[11 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3MuxCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output [31:0] io_mux_pins,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [31:0] muxCtrl_1__io_mux_pins;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [31:0] _zz_1_;
  MuxCtrl muxCtrl_1_ ( 
    .io_mux_pins(muxCtrl_1__io_mux_pins),
    .io_signals(_zz_1_) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[31 : 0] = _zz_1_;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_mux_pins = muxCtrl_1__io_mux_pins;
  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[31 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3MachineTimerCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [31:0] mtCtrl_io_micros;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  MachineTimerCtrl mtCtrl ( 
    .io_micros(mtCtrl_io_micros),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[31 : 0] = mtCtrl_io_micros;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
endmodule

module Apb3ToneCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output  io_tone_pin,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  toneCtrl_1__io_tone_pin;
  wire  toneCtrl_1__io_done;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg  _zz_1_;
  reg [31:0] _zz_2_;
  reg [31:0] _zz_3_;
  reg  _zz_4_;
  ToneCtrl toneCtrl_1_ ( 
    .io_tone_pin(toneCtrl_1__io_tone_pin),
    .io_period(_zz_2_),
    .io_duration(_zz_3_),
    .io_clear(_zz_1_),
    .io_done(toneCtrl_1__io_done),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  assign io_apb_PRDATA = (32'b00000000000000000000000000000000);
  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_tone_pin = toneCtrl_1__io_tone_pin;
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(_zz_4_)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= io_apb_PWDATA[31 : 0];
        end
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= io_apb_PWDATA[31 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3ShiftOutCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output  io_shiftOut_dataPin,
      output  io_shiftOut_clockPin,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  shiftOutCtrl_1__io_shiftOut_dataPin;
  wire  shiftOutCtrl_1__io_shiftOut_clockPin;
  wire [0:0] _zz_6_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg  _zz_1_;
  reg [7:0] _zz_2_;
  reg  _zz_3_;
  reg [31:0] _zz_4_;
  reg  _zz_5_;
  assign _zz_6_ = io_apb_PWDATA[0 : 0];
  ShiftOutCtrl shiftOutCtrl_1_ ( 
    .io_shiftOut_dataPin(shiftOutCtrl_1__io_shiftOut_dataPin),
    .io_shiftOut_clockPin(shiftOutCtrl_1__io_shiftOut_clockPin),
    .io_bitOrder(_zz_3_),
    .io_value(_zz_2_),
    .io_preScale(_zz_4_),
    .io_set(_zz_1_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  assign io_apb_PRDATA = (32'b00000000000000000000000000000000);
  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_shiftOut_dataPin = shiftOutCtrl_1__io_shiftOut_dataPin;
  assign io_shiftOut_clockPin = shiftOutCtrl_1__io_shiftOut_clockPin;
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(_zz_5_)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_5_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_5_ = 1'b1;
        end
      end
      8'b00000100 : begin
      end
      8'b00001000 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= io_apb_PWDATA[7 : 0];
        end
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= _zz_6_[0];
        end
      end
      8'b00001000 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ <= io_apb_PWDATA[31 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3SpiMasterCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output [0:0] io_spi_ss,
      output  io_spi_sclk,
      output  io_spi_mosi,
      input   io_spi_miso,
      output  io_interrupt,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  _zz_11_;
  reg  _zz_12_;
  wire  _zz_13_;
  wire  spiCtrl_io_cmd_ready;
  wire  spiCtrl_io_rsp_valid;
  wire [7:0] spiCtrl_io_rsp_payload;
  wire  spiCtrl_io_spi_sclk;
  wire  spiCtrl_io_spi_mosi;
  wire [0:0] spiCtrl_io_spi_ss;
  wire  bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready;
  wire  bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_mode;
  wire [8:0] bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_args;
  wire [5:0] bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_occupancy;
  wire [5:0] bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability;
  wire  spiCtrl_io_rsp_queueWithOccupancy_io_push_ready;
  wire  spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid;
  wire [7:0] spiCtrl_io_rsp_queueWithOccupancy_io_pop_payload;
  wire [5:0] spiCtrl_io_rsp_queueWithOccupancy_io_occupancy;
  wire [5:0] spiCtrl_io_rsp_queueWithOccupancy_io_availability;
  wire [0:0] _zz_14_;
  wire [0:0] _zz_15_;
  wire [0:0] _zz_16_;
  wire [0:0] _zz_17_;
  wire [0:0] _zz_18_;
  wire [0:0] _zz_19_;
  wire [0:0] _zz_20_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  wire  bridge_cmdLogic_streamUnbuffered_valid;
  wire  bridge_cmdLogic_streamUnbuffered_ready;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type bridge_cmdLogic_streamUnbuffered_payload_mode;
  reg [8:0] bridge_cmdLogic_streamUnbuffered_payload_args;
  reg  _zz_1_;
  wire [7:0] bridge_cmdLogic_dataCmd_data;
  wire  bridge_cmdLogic_dataCmd_read;
  reg  bridge_interruptCtrl_cmdIntEnable;
  reg  bridge_interruptCtrl_rspIntEnable;
  wire  bridge_interruptCtrl_cmdInt;
  wire  bridge_interruptCtrl_rspInt;
  wire  bridge_interruptCtrl_interrupt;
  reg  _zz_2_;
  reg  _zz_3_;
  reg [31:0] _zz_4_;
  reg [0:0] _zz_5_;
  reg [31:0] _zz_6_;
  reg [31:0] _zz_7_;
  reg [31:0] _zz_8_;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type _zz_9_;
  wire [1:0] _zz_10_;
  `ifndef SYNTHESIS
  reg [31:0] bridge_cmdLogic_streamUnbuffered_payload_mode_string;
  reg [31:0] _zz_9__string;
  `endif

  assign _zz_14_ = _zz_15_[0];
  assign _zz_15_ = io_apb_PWDATA[24 : 24];
  assign _zz_16_ = io_apb_PWDATA[24 : 24];
  assign _zz_17_ = io_apb_PWDATA[0 : 0];
  assign _zz_18_ = io_apb_PWDATA[1 : 1];
  assign _zz_19_ = _zz_10_[0 : 0];
  assign _zz_20_ = _zz_10_[1 : 1];
  SpiMasterCtrl spiCtrl ( 
    .io_config_kind_cpol(_zz_2_),
    .io_config_kind_cpha(_zz_3_),
    .io_config_sclkToogle(_zz_4_),
    .io_config_ss_activeHigh(_zz_5_),
    .io_config_ss_setup(_zz_6_),
    .io_config_ss_hold(_zz_7_),
    .io_config_ss_disable(_zz_8_),
    .io_cmd_valid(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid),
    .io_cmd_ready(spiCtrl_io_cmd_ready),
    .io_cmd_payload_mode(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_mode),
    .io_cmd_payload_args(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_args),
    .io_rsp_valid(spiCtrl_io_rsp_valid),
    .io_rsp_payload(spiCtrl_io_rsp_payload),
    .io_spi_ss(spiCtrl_io_spi_ss),
    .io_spi_sclk(spiCtrl_io_spi_sclk),
    .io_spi_mosi(spiCtrl_io_spi_mosi),
    .io_spi_miso(io_spi_miso),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  StreamFifo_2_ bridge_cmdLogic_streamUnbuffered_queueWithAvailability ( 
    .io_push_valid(bridge_cmdLogic_streamUnbuffered_valid),
    .io_push_ready(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready),
    .io_push_payload_mode(bridge_cmdLogic_streamUnbuffered_payload_mode),
    .io_push_payload_args(bridge_cmdLogic_streamUnbuffered_payload_args),
    .io_pop_valid(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid),
    .io_pop_ready(spiCtrl_io_cmd_ready),
    .io_pop_payload_mode(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_mode),
    .io_pop_payload_args(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_args),
    .io_flush(_zz_11_),
    .io_occupancy(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_occupancy),
    .io_availability(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  StreamFifo_3_ spiCtrl_io_rsp_queueWithOccupancy ( 
    .io_push_valid(spiCtrl_io_rsp_valid),
    .io_push_ready(spiCtrl_io_rsp_queueWithOccupancy_io_push_ready),
    .io_push_payload(spiCtrl_io_rsp_payload),
    .io_pop_valid(spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(_zz_12_),
    .io_pop_payload(spiCtrl_io_rsp_queueWithOccupancy_io_pop_payload),
    .io_flush(_zz_13_),
    .io_occupancy(spiCtrl_io_rsp_queueWithOccupancy_io_occupancy),
    .io_availability(spiCtrl_io_rsp_queueWithOccupancy_io_availability),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(bridge_cmdLogic_streamUnbuffered_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : bridge_cmdLogic_streamUnbuffered_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : bridge_cmdLogic_streamUnbuffered_payload_mode_string = "SS  ";
      default : bridge_cmdLogic_streamUnbuffered_payload_mode_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : _zz_9__string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : _zz_9__string = "SS  ";
      default : _zz_9__string = "????";
    endcase
  end
  `endif

  assign io_spi_ss = spiCtrl_io_spi_ss;
  assign io_spi_sclk = spiCtrl_io_spi_sclk;
  assign io_spi_mosi = spiCtrl_io_spi_mosi;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_1_ = 1'b0;
    _zz_12_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ = 1'b1;
        end
        if(busCtrl_doRead)begin
          _zz_12_ = 1'b1;
        end
        io_apb_PRDATA[31 : 31] = spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid;
        io_apb_PRDATA[7 : 0] = spiCtrl_io_rsp_queueWithOccupancy_io_pop_payload;
        io_apb_PRDATA[21 : 16] = spiCtrl_io_rsp_queueWithOccupancy_io_occupancy;
      end
      8'b00000100 : begin
        io_apb_PRDATA[21 : 16] = bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability;
        io_apb_PRDATA[0 : 0] = bridge_interruptCtrl_cmdIntEnable;
        io_apb_PRDATA[1 : 1] = bridge_interruptCtrl_rspIntEnable;
        io_apb_PRDATA[8 : 8] = bridge_interruptCtrl_cmdInt;
        io_apb_PRDATA[9 : 9] = bridge_interruptCtrl_rspInt;
      end
      8'b00001000 : begin
      end
      8'b00001100 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      8'b00011000 : begin
      end
      default : begin
      end
    endcase
  end

  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign bridge_cmdLogic_streamUnbuffered_valid = _zz_1_;
  always @ (*) begin
    case(bridge_cmdLogic_streamUnbuffered_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : begin
        bridge_cmdLogic_streamUnbuffered_payload_args = {bridge_cmdLogic_dataCmd_read,bridge_cmdLogic_dataCmd_data};
      end
      default : begin
        bridge_cmdLogic_streamUnbuffered_payload_args = {8'd0, _zz_14_};
      end
    endcase
  end

  assign bridge_cmdLogic_streamUnbuffered_ready = bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready;
  assign bridge_interruptCtrl_cmdInt = (bridge_interruptCtrl_cmdIntEnable && (! bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid));
  assign bridge_interruptCtrl_rspInt = (bridge_interruptCtrl_rspIntEnable && spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid);
  assign bridge_interruptCtrl_interrupt = (bridge_interruptCtrl_rspInt || bridge_interruptCtrl_cmdInt);
  assign io_interrupt = bridge_interruptCtrl_interrupt;
  assign bridge_cmdLogic_dataCmd_data = io_apb_PWDATA[7 : 0];
  assign bridge_cmdLogic_dataCmd_read = _zz_16_[0];
  assign _zz_9_ = io_apb_PWDATA[28 : 28];
  assign bridge_cmdLogic_streamUnbuffered_payload_mode = _zz_9_;
  assign _zz_10_ = io_apb_PWDATA[1 : 0];
  assign _zz_11_ = 1'b0;
  assign _zz_13_ = 1'b0;
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      bridge_interruptCtrl_cmdIntEnable <= 1'b0;
      bridge_interruptCtrl_rspIntEnable <= 1'b0;
      _zz_5_ <= (1'b0);
    end else begin
      case(io_apb_PADDR)
        8'b00000000 : begin
        end
        8'b00000100 : begin
          if(busCtrl_doWrite)begin
            bridge_interruptCtrl_cmdIntEnable <= _zz_17_[0];
            bridge_interruptCtrl_rspIntEnable <= _zz_18_[0];
          end
        end
        8'b00001000 : begin
          if(busCtrl_doWrite)begin
            _zz_5_ <= io_apb_PWDATA[4 : 4];
          end
        end
        8'b00001100 : begin
        end
        8'b00010000 : begin
        end
        8'b00010100 : begin
        end
        8'b00011000 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b00000100 : begin
      end
      8'b00001000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= _zz_19_[0];
          _zz_3_ <= _zz_20_[0];
        end
      end
      8'b00001100 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ <= io_apb_PWDATA[31 : 0];
        end
      end
      8'b00010000 : begin
        if(busCtrl_doWrite)begin
          _zz_6_ <= io_apb_PWDATA[31 : 0];
        end
      end
      8'b00010100 : begin
        if(busCtrl_doWrite)begin
          _zz_7_ <= io_apb_PWDATA[31 : 0];
        end
      end
      8'b00011000 : begin
        if(busCtrl_doWrite)begin
          _zz_8_ <= io_apb_PWDATA[31 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3I2cCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_i2c_sda_write,
      input   io_i2c_sda_read,
      output  io_i2c_scl_write,
      input   io_i2c_scl_read,
      output  io_interrupt,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg  _zz_14_;
  reg  _zz_15_;
  reg  _zz_16_;
  wire  i2cCtrl_io_i2c_scl_write;
  wire  i2cCtrl_io_i2c_sda_write;
  wire `I2cSlaveCmdMode_defaultEncoding_type i2cCtrl_io_bus_cmd_kind;
  wire  i2cCtrl_io_bus_cmd_data;
  wire  i2cCtrl_io_internals_inFrame;
  wire  i2cCtrl_io_internals_sdaRead;
  wire  i2cCtrl_io_internals_sclRead;
  wire  _zz_17_;
  wire  _zz_18_;
  wire  _zz_19_;
  wire [0:0] _zz_20_;
  wire [11:0] _zz_21_;
  wire [2:0] _zz_22_;
  wire [2:0] _zz_23_;
  wire [0:0] _zz_24_;
  wire [0:0] _zz_25_;
  wire [0:0] _zz_26_;
  wire [0:0] _zz_27_;
  wire [0:0] _zz_28_;
  wire [0:0] _zz_29_;
  wire [0:0] _zz_30_;
  wire [0:0] _zz_31_;
  wire [0:0] _zz_32_;
  wire [0:0] _zz_33_;
  wire [0:0] _zz_34_;
  wire [0:0] _zz_35_;
  wire [0:0] _zz_36_;
  wire [0:0] _zz_37_;
  wire [0:0] _zz_38_;
  wire [0:0] _zz_39_;
  wire [0:0] _zz_40_;
  wire [0:0] _zz_41_;
  wire [0:0] _zz_42_;
  wire [0:0] _zz_43_;
  wire [0:0] _zz_44_;
  wire [0:0] _zz_45_;
  wire [0:0] _zz_46_;
  wire [0:0] _zz_47_;
  wire [0:0] _zz_48_;
  wire [0:0] _zz_49_;
  wire [0:0] _zz_50_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg  bridge_frameReset;
  reg  bridge_i2cBuffer_sda_write;
  wire  bridge_i2cBuffer_sda_read;
  reg  bridge_i2cBuffer_scl_write;
  wire  bridge_i2cBuffer_scl_read;
  reg  bridge_rxData_event;
  reg  bridge_rxData_listen;
  reg  bridge_rxData_valid;
  reg [7:0] bridge_rxData_value;
  reg  _zz_1_;
  reg  bridge_rxAck_listen;
  reg  bridge_rxAck_valid;
  reg  bridge_rxAck_value;
  reg  _zz_2_;
  reg  bridge_txData_valid;
  reg  bridge_txData_repeat;
  reg  bridge_txData_enable;
  reg [7:0] bridge_txData_value;
  reg  bridge_txData_forceDisable;
  reg  bridge_txData_disableOnDataConflict;
  reg  bridge_txAck_valid;
  reg  bridge_txAck_repeat;
  reg  bridge_txAck_enable;
  reg  bridge_txAck_value;
  wire  bridge_txAck_forceAck;
  reg  bridge_txAck_disableOnDataConflict;
  reg  bridge_masterLogic_start;
  reg  bridge_masterLogic_stop;
  reg  bridge_masterLogic_drop;
  reg [11:0] bridge_masterLogic_timer_value;
  reg [11:0] bridge_masterLogic_timer_tLow;
  reg [11:0] bridge_masterLogic_timer_tHigh;
  reg [11:0] bridge_masterLogic_timer_tBuf;
  wire  bridge_masterLogic_timer_done;
  wire  bridge_masterLogic_txReady;
  wire  bridge_masterLogic_fsm_wantExit;
  wire  bridge_isBusy;
  reg [2:0] bridge_dataCounter;
  reg  bridge_inAckState;
  reg  bridge_wasntAck;
  reg  bridge_interruptCtrl_rxDataEnable;
  reg  bridge_interruptCtrl_rxAckEnable;
  reg  bridge_interruptCtrl_txDataEnable;
  reg  bridge_interruptCtrl_txAckEnable;
  reg  bridge_interruptCtrl_start_enable;
  reg  bridge_interruptCtrl_start_flag;
  reg  _zz_3_;
  reg  bridge_interruptCtrl_restart_enable;
  reg  bridge_interruptCtrl_restart_flag;
  reg  _zz_4_;
  reg  bridge_interruptCtrl_end_enable;
  reg  bridge_interruptCtrl_end_flag;
  reg  _zz_5_;
  reg  bridge_interruptCtrl_drop_enable;
  reg  bridge_interruptCtrl_drop_flag;
  reg  _zz_6_;
  reg  bridge_interruptCtrl_interrupt;
  reg  bridge_interruptCtrl_clockGen_busyEnable;
  reg [9:0] _zz_7_;
  reg [19:0] _zz_8_ = 20'b00000000000000000000;
  reg [5:0] _zz_9_ = 6'b000000;
  wire [0:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [0:0] _zz_12_;
  wire [0:0] _zz_13_;
  reg `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_type bridge_masterLogic_fsm_stateReg;
  reg `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_type bridge_masterLogic_fsm_stateNext;
  `ifndef SYNTHESIS
  reg [239:0] bridge_masterLogic_fsm_stateReg_string;
  reg [239:0] bridge_masterLogic_fsm_stateNext_string;
  `endif

  assign _zz_17_ = ((i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_STOP) || (i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_DROP));
  assign _zz_18_ = (bridge_masterLogic_timer_done || (! i2cCtrl_io_internals_sclRead));
  assign _zz_19_ = (! bridge_inAckState);
  assign _zz_20_ = (! bridge_masterLogic_timer_done);
  assign _zz_21_ = {11'd0, _zz_20_};
  assign _zz_22_ = ((3'b111) - bridge_dataCounter);
  assign _zz_23_ = ((3'b111) - bridge_dataCounter);
  assign _zz_24_ = (1'b0);
  assign _zz_25_ = (1'b0);
  assign _zz_26_ = (1'b0);
  assign _zz_27_ = (1'b0);
  assign _zz_28_ = io_apb_PWDATA[9 : 9];
  assign _zz_29_ = io_apb_PWDATA[9 : 9];
  assign _zz_30_ = io_apb_PWDATA[10 : 10];
  assign _zz_31_ = io_apb_PWDATA[11 : 11];
  assign _zz_32_ = io_apb_PWDATA[8 : 8];
  assign _zz_33_ = io_apb_PWDATA[9 : 9];
  assign _zz_34_ = io_apb_PWDATA[0 : 0];
  assign _zz_35_ = io_apb_PWDATA[10 : 10];
  assign _zz_36_ = io_apb_PWDATA[11 : 11];
  assign _zz_37_ = io_apb_PWDATA[8 : 8];
  assign _zz_38_ = io_apb_PWDATA[9 : 9];
  assign _zz_39_ = io_apb_PWDATA[4 : 4];
  assign _zz_40_ = io_apb_PWDATA[5 : 5];
  assign _zz_41_ = io_apb_PWDATA[6 : 6];
  assign _zz_42_ = io_apb_PWDATA[0 : 0];
  assign _zz_43_ = io_apb_PWDATA[1 : 1];
  assign _zz_44_ = io_apb_PWDATA[2 : 2];
  assign _zz_45_ = io_apb_PWDATA[3 : 3];
  assign _zz_46_ = io_apb_PWDATA[4 : 4];
  assign _zz_47_ = io_apb_PWDATA[5 : 5];
  assign _zz_48_ = io_apb_PWDATA[6 : 6];
  assign _zz_49_ = io_apb_PWDATA[7 : 7];
  assign _zz_50_ = io_apb_PWDATA[16 : 16];
  I2cSlave i2cCtrl ( 
    .io_i2c_sda_write(i2cCtrl_io_i2c_sda_write),
    .io_i2c_sda_read(bridge_i2cBuffer_sda_read),
    .io_i2c_scl_write(i2cCtrl_io_i2c_scl_write),
    .io_i2c_scl_read(bridge_i2cBuffer_scl_read),
    .io_config_samplingClockDivider(_zz_7_),
    .io_config_timeout(_zz_8_),
    .io_config_tsuData(_zz_9_),
    .io_bus_cmd_kind(i2cCtrl_io_bus_cmd_kind),
    .io_bus_cmd_data(i2cCtrl_io_bus_cmd_data),
    .io_bus_rsp_valid(_zz_14_),
    .io_bus_rsp_enable(_zz_15_),
    .io_bus_rsp_data(_zz_16_),
    .io_internals_inFrame(i2cCtrl_io_internals_inFrame),
    .io_internals_sdaRead(i2cCtrl_io_internals_sdaRead),
    .io_internals_sclRead(i2cCtrl_io_internals_sclRead),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(bridge_masterLogic_fsm_stateReg)
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_boot : bridge_masterLogic_fsm_stateReg_string = "boot                          ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_IDLE   ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_START  ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_LOW    ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_HIGH   ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_RESTART";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1 : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_STOP1  ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2 : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_STOP2  ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF : bridge_masterLogic_fsm_stateReg_string = "bridge_masterLogic_fsm_TBUF   ";
      default : bridge_masterLogic_fsm_stateReg_string = "??????????????????????????????";
    endcase
  end
  always @(*) begin
    case(bridge_masterLogic_fsm_stateNext)
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_boot : bridge_masterLogic_fsm_stateNext_string = "boot                          ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_IDLE   ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_START  ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_LOW    ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_HIGH   ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_RESTART";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1 : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_STOP1  ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2 : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_STOP2  ";
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF : bridge_masterLogic_fsm_stateNext_string = "bridge_masterLogic_fsm_TBUF   ";
      default : bridge_masterLogic_fsm_stateNext_string = "??????????????????????????????";
    endcase
  end
  `endif

  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_1_ = 1'b0;
    _zz_2_ = 1'b0;
    _zz_3_ = 1'b0;
    _zz_4_ = 1'b0;
    _zz_5_ = 1'b0;
    _zz_6_ = 1'b0;
    case(io_apb_PADDR)
      8'b00001000 : begin
        if(busCtrl_doRead)begin
          _zz_1_ = 1'b1;
        end
        io_apb_PRDATA[8 : 8] = bridge_rxData_valid;
        io_apb_PRDATA[7 : 0] = bridge_rxData_value;
      end
      8'b00001100 : begin
        if(busCtrl_doRead)begin
          _zz_2_ = 1'b1;
        end
        io_apb_PRDATA[8 : 8] = bridge_rxAck_valid;
        io_apb_PRDATA[0 : 0] = bridge_rxAck_value;
      end
      8'b00000000 : begin
        io_apb_PRDATA[8 : 8] = bridge_txData_valid;
        io_apb_PRDATA[9 : 9] = bridge_txData_enable;
      end
      8'b00000100 : begin
        io_apb_PRDATA[8 : 8] = bridge_txAck_valid;
        io_apb_PRDATA[9 : 9] = bridge_txAck_enable;
      end
      8'b01000000 : begin
        io_apb_PRDATA[4 : 4] = bridge_masterLogic_start;
        io_apb_PRDATA[5 : 5] = bridge_masterLogic_stop;
        io_apb_PRDATA[6 : 6] = bridge_masterLogic_drop;
        io_apb_PRDATA[0 : 0] = bridge_isBusy;
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
      end
      8'b01011000 : begin
      end
      8'b00100000 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ = 1'b1;
          _zz_4_ = 1'b1;
          _zz_5_ = 1'b1;
          _zz_6_ = 1'b1;
        end
        io_apb_PRDATA[0 : 0] = bridge_interruptCtrl_rxDataEnable;
        io_apb_PRDATA[1 : 1] = bridge_interruptCtrl_rxAckEnable;
        io_apb_PRDATA[2 : 2] = bridge_interruptCtrl_txDataEnable;
        io_apb_PRDATA[3 : 3] = bridge_interruptCtrl_txAckEnable;
        io_apb_PRDATA[4 : 4] = bridge_interruptCtrl_start_enable;
        io_apb_PRDATA[8 : 8] = bridge_interruptCtrl_start_flag;
        io_apb_PRDATA[5 : 5] = bridge_interruptCtrl_restart_enable;
        io_apb_PRDATA[9 : 9] = bridge_interruptCtrl_restart_flag;
        io_apb_PRDATA[6 : 6] = bridge_interruptCtrl_end_enable;
        io_apb_PRDATA[10 : 10] = bridge_interruptCtrl_end_flag;
        io_apb_PRDATA[7 : 7] = bridge_interruptCtrl_drop_enable;
        io_apb_PRDATA[11 : 11] = bridge_interruptCtrl_drop_flag;
        io_apb_PRDATA[16 : 16] = bridge_interruptCtrl_clockGen_busyEnable;
      end
      8'b00101000 : begin
      end
      8'b00101100 : begin
      end
      8'b00110000 : begin
      end
      default : begin
      end
    endcase
  end

  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    bridge_frameReset = 1'b0;
    case(i2cCtrl_io_bus_cmd_kind)
      `I2cSlaveCmdMode_defaultEncoding_START : begin
        bridge_frameReset = 1'b1;
      end
      `I2cSlaveCmdMode_defaultEncoding_RESTART : begin
        bridge_frameReset = 1'b1;
      end
      `I2cSlaveCmdMode_defaultEncoding_STOP : begin
        bridge_frameReset = 1'b1;
      end
      `I2cSlaveCmdMode_defaultEncoding_DROP : begin
        bridge_frameReset = 1'b1;
      end
      `I2cSlaveCmdMode_defaultEncoding_DRIVE : begin
      end
      `I2cSlaveCmdMode_defaultEncoding_READ : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    bridge_i2cBuffer_sda_write = i2cCtrl_io_i2c_sda_write;
    bridge_i2cBuffer_scl_write = i2cCtrl_io_i2c_scl_write;
    bridge_txData_forceDisable = 1'b0;
    if(_zz_17_)begin
      bridge_txData_forceDisable = 1'b0;
    end
    bridge_masterLogic_fsm_stateNext = bridge_masterLogic_fsm_stateReg;
    case(bridge_masterLogic_fsm_stateReg)
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE : begin
        if((bridge_masterLogic_start && (! i2cCtrl_io_internals_inFrame)))begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START : begin
        bridge_i2cBuffer_sda_write = 1'b0;
        if(_zz_18_)begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW : begin
        if(bridge_masterLogic_timer_done)begin
          if((bridge_masterLogic_stop && (! bridge_inAckState)))begin
            bridge_txData_forceDisable = 1'b1;
            bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1;
          end else begin
            if((bridge_masterLogic_start && (! bridge_inAckState)))begin
              bridge_txData_forceDisable = 1'b1;
              bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART;
            end else begin
              if(i2cCtrl_io_internals_sclRead)begin
                bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH;
              end
            end
          end
        end else begin
          bridge_i2cBuffer_scl_write = 1'b0;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH : begin
        if((bridge_masterLogic_timer_done || (! i2cCtrl_io_internals_sclRead)))begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART : begin
        if(bridge_masterLogic_timer_done)begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1 : begin
        bridge_i2cBuffer_scl_write = 1'b0;
        bridge_i2cBuffer_sda_write = 1'b0;
        if(bridge_masterLogic_timer_done)begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2 : begin
        bridge_i2cBuffer_sda_write = 1'b0;
        if(bridge_masterLogic_timer_done)begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF : begin
        if(bridge_masterLogic_timer_done)begin
          bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE;
        end
      end
      default : begin
        bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE;
      end
    endcase
    if(bridge_masterLogic_drop)begin
      bridge_masterLogic_fsm_stateNext = `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF;
    end
  end

  assign bridge_txAck_forceAck = 1'b0;
  assign bridge_masterLogic_timer_done = (bridge_masterLogic_timer_value == (12'b000000000000));
  assign bridge_masterLogic_fsm_wantExit = 1'b0;
  assign bridge_isBusy = (! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE));
  assign bridge_masterLogic_txReady = (bridge_inAckState ? bridge_txAck_valid : bridge_txData_valid);
  always @ (*) begin
    if((! bridge_inAckState))begin
      _zz_14_ = ((bridge_txData_valid && (! (bridge_rxData_valid && bridge_rxData_listen))) && (i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_DRIVE));
      _zz_15_ = bridge_txData_enable;
      _zz_16_ = (bridge_masterLogic_start ? 1'b1 : bridge_txData_value[_zz_22_]);
      if(bridge_txData_forceDisable)begin
        _zz_14_ = 1'b1;
        _zz_15_ = 1'b0;
      end
    end else begin
      _zz_14_ = ((bridge_txAck_valid && (! (bridge_rxAck_valid && bridge_rxAck_listen))) && (i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_DRIVE));
      _zz_15_ = bridge_txAck_enable;
      _zz_16_ = bridge_txAck_value;
      if(bridge_txAck_forceAck)begin
        _zz_14_ = 1'b1;
        _zz_15_ = 1'b1;
        _zz_16_ = 1'b0;
      end
    end
    if((bridge_wasntAck && (! bridge_isBusy)))begin
      _zz_14_ = (i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_DRIVE);
      _zz_15_ = 1'b0;
    end
  end

  always @ (*) begin
    bridge_interruptCtrl_interrupt = (((((bridge_interruptCtrl_rxDataEnable && bridge_rxData_valid) || (bridge_interruptCtrl_rxAckEnable && bridge_rxAck_valid)) || (bridge_interruptCtrl_txDataEnable && (! bridge_txData_valid))) || (bridge_interruptCtrl_txAckEnable && (! bridge_txAck_valid))) || (((bridge_interruptCtrl_start_flag || bridge_interruptCtrl_restart_flag) || bridge_interruptCtrl_end_flag) || bridge_interruptCtrl_drop_flag));
    if((bridge_interruptCtrl_clockGen_busyEnable && bridge_isBusy))begin
      bridge_interruptCtrl_interrupt = 1'b1;
    end
  end

  assign io_i2c_sda_write = bridge_i2cBuffer_sda_write;
  assign bridge_i2cBuffer_sda_read = io_i2c_sda_read;
  assign io_i2c_scl_write = bridge_i2cBuffer_scl_write;
  assign bridge_i2cBuffer_scl_read = io_i2c_scl_read;
  assign io_interrupt = bridge_interruptCtrl_interrupt;
  assign _zz_10_ = io_apb_PWDATA[12 : 12];
  assign _zz_11_ = io_apb_PWDATA[13 : 13];
  assign _zz_12_ = io_apb_PWDATA[14 : 14];
  assign _zz_13_ = io_apb_PWDATA[15 : 15];
  always @ (posedge toplevel_io_mainClk or posedge toplevel_resetCtrl_systemReset) begin
    if (toplevel_resetCtrl_systemReset) begin
      bridge_rxData_event <= 1'b0;
      bridge_rxData_listen <= 1'b0;
      bridge_rxData_valid <= 1'b0;
      bridge_rxAck_listen <= 1'b0;
      bridge_rxAck_valid <= 1'b0;
      bridge_txData_valid <= 1'b1;
      bridge_txData_repeat <= 1'b1;
      bridge_txData_enable <= 1'b0;
      bridge_txAck_valid <= 1'b1;
      bridge_txAck_repeat <= 1'b1;
      bridge_txAck_enable <= 1'b0;
      bridge_masterLogic_start <= 1'b0;
      bridge_masterLogic_stop <= 1'b0;
      bridge_masterLogic_drop <= 1'b0;
      bridge_dataCounter <= (3'b000);
      bridge_inAckState <= 1'b0;
      bridge_wasntAck <= 1'b0;
      bridge_interruptCtrl_rxDataEnable <= 1'b0;
      bridge_interruptCtrl_rxAckEnable <= 1'b0;
      bridge_interruptCtrl_txDataEnable <= 1'b0;
      bridge_interruptCtrl_txAckEnable <= 1'b0;
      bridge_interruptCtrl_start_enable <= 1'b0;
      bridge_interruptCtrl_start_flag <= 1'b0;
      bridge_interruptCtrl_restart_enable <= 1'b0;
      bridge_interruptCtrl_restart_flag <= 1'b0;
      bridge_interruptCtrl_end_enable <= 1'b0;
      bridge_interruptCtrl_end_flag <= 1'b0;
      bridge_interruptCtrl_drop_enable <= 1'b0;
      bridge_interruptCtrl_drop_flag <= 1'b0;
      bridge_interruptCtrl_clockGen_busyEnable <= 1'b0;
      _zz_7_ <= (10'b0000000000);
      bridge_masterLogic_fsm_stateReg <= `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_boot;
    end else begin
      bridge_rxData_event <= 1'b0;
      if(_zz_1_)begin
        bridge_rxData_valid <= 1'b0;
      end
      if(_zz_2_)begin
        bridge_rxAck_valid <= 1'b0;
      end
      case(i2cCtrl_io_bus_cmd_kind)
        `I2cSlaveCmdMode_defaultEncoding_START : begin
        end
        `I2cSlaveCmdMode_defaultEncoding_RESTART : begin
        end
        `I2cSlaveCmdMode_defaultEncoding_STOP : begin
        end
        `I2cSlaveCmdMode_defaultEncoding_DROP : begin
          bridge_rxData_listen <= 1'b0;
          bridge_rxAck_listen <= 1'b0;
        end
        `I2cSlaveCmdMode_defaultEncoding_DRIVE : begin
          if((! bridge_inAckState))begin
            if((bridge_txData_valid && (bridge_dataCounter == (3'b111))))begin
              if((! bridge_txData_repeat))begin
                bridge_txData_valid <= 1'b0;
              end
            end
          end
        end
        `I2cSlaveCmdMode_defaultEncoding_READ : begin
          if(_zz_19_)begin
            bridge_dataCounter <= (bridge_dataCounter + (3'b001));
            if((_zz_16_ != i2cCtrl_io_bus_cmd_data))begin
              if(bridge_txData_disableOnDataConflict)begin
                bridge_txData_enable <= 1'b0;
              end
              if(bridge_txAck_disableOnDataConflict)begin
                bridge_txAck_enable <= 1'b0;
              end
            end
            if((bridge_dataCounter == (3'b111)))begin
              if(bridge_rxData_listen)begin
                bridge_rxData_valid <= 1'b1;
              end
              bridge_rxData_event <= 1'b1;
              bridge_inAckState <= 1'b1;
            end
          end else begin
            if(bridge_rxAck_listen)begin
              bridge_rxAck_valid <= 1'b1;
            end
            bridge_inAckState <= 1'b0;
            bridge_wasntAck <= i2cCtrl_io_bus_cmd_data;
            if(bridge_txAck_valid)begin
              if((! bridge_txAck_repeat))begin
                bridge_txAck_valid <= 1'b0;
              end
            end
          end
        end
        default : begin
        end
      endcase
      if(bridge_frameReset)begin
        bridge_inAckState <= 1'b0;
        bridge_dataCounter <= (3'b000);
        bridge_wasntAck <= 1'b0;
      end
      if(_zz_17_)begin
        bridge_txData_valid <= 1'b1;
        bridge_txData_enable <= 1'b0;
        bridge_txData_repeat <= 1'b1;
        bridge_txAck_valid <= 1'b1;
        bridge_txAck_enable <= 1'b0;
        bridge_txAck_repeat <= 1'b1;
        bridge_rxData_listen <= 1'b0;
        bridge_rxAck_listen <= 1'b0;
      end
      if((i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_START))begin
        bridge_interruptCtrl_start_flag <= 1'b1;
      end
      if((! bridge_interruptCtrl_start_enable))begin
        bridge_interruptCtrl_start_flag <= 1'b0;
      end
      if(_zz_3_)begin
        if(_zz_10_[0])begin
          bridge_interruptCtrl_start_flag <= _zz_24_[0];
        end
      end
      if((i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_RESTART))begin
        bridge_interruptCtrl_restart_flag <= 1'b1;
      end
      if((! bridge_interruptCtrl_restart_enable))begin
        bridge_interruptCtrl_restart_flag <= 1'b0;
      end
      if(_zz_4_)begin
        if(_zz_11_[0])begin
          bridge_interruptCtrl_restart_flag <= _zz_25_[0];
        end
      end
      if((i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_STOP))begin
        bridge_interruptCtrl_end_flag <= 1'b1;
      end
      if((! bridge_interruptCtrl_end_enable))begin
        bridge_interruptCtrl_end_flag <= 1'b0;
      end
      if(_zz_5_)begin
        if(_zz_12_[0])begin
          bridge_interruptCtrl_end_flag <= _zz_26_[0];
        end
      end
      if((i2cCtrl_io_bus_cmd_kind == `I2cSlaveCmdMode_defaultEncoding_DROP))begin
        bridge_interruptCtrl_drop_flag <= 1'b1;
      end
      if((! bridge_interruptCtrl_drop_enable))begin
        bridge_interruptCtrl_drop_flag <= 1'b0;
      end
      if(_zz_6_)begin
        if(_zz_13_[0])begin
          bridge_interruptCtrl_drop_flag <= _zz_27_[0];
        end
      end
      case(io_apb_PADDR)
        8'b00001000 : begin
          if(busCtrl_doWrite)begin
            bridge_rxData_listen <= _zz_28_[0];
          end
        end
        8'b00001100 : begin
          if(busCtrl_doWrite)begin
            bridge_rxAck_listen <= _zz_29_[0];
          end
        end
        8'b00000000 : begin
          if(busCtrl_doWrite)begin
            bridge_txData_repeat <= _zz_30_[0];
            bridge_txData_valid <= _zz_32_[0];
            bridge_txData_enable <= _zz_33_[0];
          end
        end
        8'b00000100 : begin
          if(busCtrl_doWrite)begin
            bridge_txAck_repeat <= _zz_35_[0];
            bridge_txAck_valid <= _zz_37_[0];
            bridge_txAck_enable <= _zz_38_[0];
          end
        end
        8'b01000000 : begin
          if(busCtrl_doWrite)begin
            bridge_masterLogic_start <= _zz_39_[0];
            bridge_masterLogic_stop <= _zz_40_[0];
            bridge_masterLogic_drop <= _zz_41_[0];
          end
        end
        8'b01010000 : begin
        end
        8'b01010100 : begin
        end
        8'b01011000 : begin
        end
        8'b00100000 : begin
          if(busCtrl_doWrite)begin
            bridge_interruptCtrl_rxDataEnable <= _zz_42_[0];
            bridge_interruptCtrl_rxAckEnable <= _zz_43_[0];
            bridge_interruptCtrl_txDataEnable <= _zz_44_[0];
            bridge_interruptCtrl_txAckEnable <= _zz_45_[0];
            bridge_interruptCtrl_start_enable <= _zz_46_[0];
            bridge_interruptCtrl_restart_enable <= _zz_47_[0];
            bridge_interruptCtrl_end_enable <= _zz_48_[0];
            bridge_interruptCtrl_drop_enable <= _zz_49_[0];
            bridge_interruptCtrl_clockGen_busyEnable <= _zz_50_[0];
          end
        end
        8'b00101000 : begin
          if(busCtrl_doWrite)begin
            _zz_7_ <= io_apb_PWDATA[9 : 0];
          end
        end
        8'b00101100 : begin
        end
        8'b00110000 : begin
        end
        default : begin
        end
      endcase
      bridge_masterLogic_fsm_stateReg <= bridge_masterLogic_fsm_stateNext;
      case(bridge_masterLogic_fsm_stateReg)
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE : begin
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START : begin
          if(_zz_18_)begin
            bridge_masterLogic_start <= 1'b0;
          end
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW : begin
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH : begin
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART : begin
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1 : begin
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2 : begin
          if(bridge_masterLogic_timer_done)begin
            bridge_masterLogic_stop <= 1'b0;
          end
        end
        `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF : begin
        end
        default : begin
        end
      endcase
      if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE)))begin
        bridge_masterLogic_start <= 1'b0;
        bridge_masterLogic_stop <= 1'b0;
        bridge_masterLogic_drop <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    bridge_masterLogic_timer_value <= (bridge_masterLogic_timer_value - _zz_21_);
    case(i2cCtrl_io_bus_cmd_kind)
      `I2cSlaveCmdMode_defaultEncoding_START : begin
      end
      `I2cSlaveCmdMode_defaultEncoding_RESTART : begin
      end
      `I2cSlaveCmdMode_defaultEncoding_STOP : begin
      end
      `I2cSlaveCmdMode_defaultEncoding_DROP : begin
      end
      `I2cSlaveCmdMode_defaultEncoding_DRIVE : begin
      end
      `I2cSlaveCmdMode_defaultEncoding_READ : begin
        if(_zz_19_)begin
          bridge_rxData_value[_zz_23_] <= i2cCtrl_io_bus_cmd_data;
        end else begin
          bridge_rxAck_value <= i2cCtrl_io_bus_cmd_data;
        end
      end
      default : begin
      end
    endcase
    if(_zz_17_)begin
      bridge_txData_disableOnDataConflict <= 1'b0;
      bridge_txAck_disableOnDataConflict <= 1'b0;
    end
    case(io_apb_PADDR)
      8'b00001000 : begin
      end
      8'b00001100 : begin
      end
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          bridge_txData_value <= io_apb_PWDATA[7 : 0];
          bridge_txData_disableOnDataConflict <= _zz_31_[0];
        end
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          bridge_txAck_value <= _zz_34_[0];
          bridge_txAck_disableOnDataConflict <= _zz_36_[0];
        end
      end
      8'b01000000 : begin
      end
      8'b01010000 : begin
        if(busCtrl_doWrite)begin
          bridge_masterLogic_timer_tLow <= io_apb_PWDATA[11 : 0];
        end
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          bridge_masterLogic_timer_tHigh <= io_apb_PWDATA[11 : 0];
        end
      end
      8'b01011000 : begin
        if(busCtrl_doWrite)begin
          bridge_masterLogic_timer_tBuf <= io_apb_PWDATA[11 : 0];
        end
      end
      8'b00100000 : begin
      end
      8'b00101000 : begin
      end
      8'b00101100 : begin
        if(busCtrl_doWrite)begin
          _zz_8_ <= io_apb_PWDATA[19 : 0];
        end
      end
      8'b00110000 : begin
        if(busCtrl_doWrite)begin
          _zz_9_ <= io_apb_PWDATA[5 : 0];
        end
      end
      default : begin
      end
    endcase
    case(bridge_masterLogic_fsm_stateReg)
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_IDLE : begin
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START : begin
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW : begin
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH : begin
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART : begin
        if((! i2cCtrl_io_internals_sclRead))begin
          bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tHigh;
        end
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1 : begin
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2 : begin
      end
      `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF : begin
      end
      default : begin
      end
    endcase
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_START)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tHigh;
    end
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_LOW)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tLow;
    end
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_HIGH)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tHigh;
    end
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_RESTART)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tHigh;
    end
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP1)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tHigh;
    end
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_STOP2)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tHigh;
    end
    if(((! (bridge_masterLogic_fsm_stateReg == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF)) && (bridge_masterLogic_fsm_stateNext == `bridge_masterLogic_fsm_enumDefinition_defaultEncoding_bridge_masterLogic_fsm_TBUF)))begin
      bridge_masterLogic_timer_value <= bridge_masterLogic_timer_tBuf;
    end
  end

endmodule

module Apb3PulseInCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input   io_pulseIn_pin,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [31:0] pulseInCtrl_1__io_pulseLength;
  wire [0:0] _zz_5_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg  _zz_1_;
  reg  _zz_2_;
  reg [31:0] _zz_3_;
  reg  _zz_4_;
  assign _zz_5_ = io_apb_PWDATA[0 : 0];
  PulseInCtrl pulseInCtrl_1_ ( 
    .io_pulseIn_pin(io_pulseIn_pin),
    .io_timeout(_zz_3_),
    .io_value(_zz_2_),
    .io_req(_zz_1_),
    .io_pulseLength(pulseInCtrl_1__io_pulseLength),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ = 1'b1;
        end
      end
      8'b00000100 : begin
      end
      8'b00001000 : begin
        io_apb_PRDATA[31 : 0] = pulseInCtrl_1__io_pulseLength;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(_zz_4_)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= _zz_5_[0];
        end
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= io_apb_PWDATA[31 : 0];
        end
      end
      8'b00001000 : begin
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3SevenSegmentCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output  io_sevenSegment_digitPin,
      output [6:0] io_sevenSegment_segPins,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  sevenSegmentCtrl_1__io_sevenSegment_digitPin;
  wire [6:0] sevenSegmentCtrl_1__io_sevenSegment_segPins;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [7:0] _zz_1_;
  SevenSegmentCtrl sevenSegmentCtrl_1_ ( 
    .io_sevenSegment_digitPin(sevenSegmentCtrl_1__io_sevenSegment_digitPin),
    .io_sevenSegment_segPins(sevenSegmentCtrl_1__io_sevenSegment_segPins),
    .io_value(_zz_1_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  assign io_apb_PRDATA = (32'b00000000000000000000000000000000);
  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_sevenSegment_digitPin = sevenSegmentCtrl_1__io_sevenSegment_digitPin;
  assign io_sevenSegment_segPins = sevenSegmentCtrl_1__io_sevenSegment_segPins;
  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[7 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3ShiftInCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input   io_shiftIn_dataPin,
      output  io_shiftIn_clockPin,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire  shiftInCtrl_1__io_shiftIn_clockPin;
  wire [7:0] shiftInCtrl_1__io_value;
  wire [0:0] _zz_5_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg  _zz_1_;
  reg [11:0] _zz_2_;
  reg  _zz_3_;
  reg  _zz_4_;
  assign _zz_5_ = io_apb_PWDATA[0 : 0];
  ShiftInCtrl shiftInCtrl_1_ ( 
    .io_shiftIn_dataPin(io_shiftIn_dataPin),
    .io_shiftIn_clockPin(shiftInCtrl_1__io_shiftIn_clockPin),
    .io_value(shiftInCtrl_1__io_value),
    .io_req(_zz_1_),
    .io_preScale(_zz_2_),
    .io_bitOrder(_zz_3_),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[7 : 0] = shiftInCtrl_1__io_value;
      end
      8'b00000100 : begin
      end
      8'b00001000 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_shiftIn_clockPin = shiftInCtrl_1__io_shiftIn_clockPin;
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(_zz_4_)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (posedge toplevel_io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b00000100 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ <= io_apb_PWDATA[11 : 0];
        end
      end
      8'b00001000 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ <= _zz_5_[0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3QspiCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input   io_qspi_qck,
      input   io_qspi_qss,
      input  [3:0] io_qspi_qd_read,
      output [3:0] io_qspi_qd_write,
      output [3:0] io_qspi_qd_writeEnable,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  wire [3:0] qspiCtrl_1__io_qspi_qd_write;
  wire [3:0] qspiCtrl_1__io_qspi_qd_writeEnable;
  wire [9:0] qspiCtrl_1__io_a0;
  wire [9:0] qspiCtrl_1__io_a1;
  wire [9:0] qspiCtrl_1__io_a2;
  wire [9:0] qspiCtrl_1__io_a3;
  wire [9:0] qspiCtrl_1__io_a4;
  wire [9:0] qspiCtrl_1__io_a5;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  QspiCtrl qspiCtrl_1_ ( 
    .io_qspi_qck(io_qspi_qck),
    .io_qspi_qss(io_qspi_qss),
    .io_qspi_qd_read(io_qspi_qd_read),
    .io_qspi_qd_write(qspiCtrl_1__io_qspi_qd_write),
    .io_qspi_qd_writeEnable(qspiCtrl_1__io_qspi_qd_writeEnable),
    .io_a0(qspiCtrl_1__io_a0),
    .io_a1(qspiCtrl_1__io_a1),
    .io_a2(qspiCtrl_1__io_a2),
    .io_a3(qspiCtrl_1__io_a3),
    .io_a4(qspiCtrl_1__io_a4),
    .io_a5(qspiCtrl_1__io_a5),
    .toplevel_io_mainClk(toplevel_io_mainClk),
    .toplevel_resetCtrl_systemReset(toplevel_resetCtrl_systemReset) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[9 : 0] = qspiCtrl_1__io_a0;
      end
      8'b00000100 : begin
        io_apb_PRDATA[9 : 0] = qspiCtrl_1__io_a1;
      end
      8'b00001000 : begin
        io_apb_PRDATA[9 : 0] = qspiCtrl_1__io_a2;
      end
      8'b00001100 : begin
        io_apb_PRDATA[9 : 0] = qspiCtrl_1__io_a3;
      end
      8'b00010000 : begin
        io_apb_PRDATA[9 : 0] = qspiCtrl_1__io_a4;
      end
      8'b00010100 : begin
        io_apb_PRDATA[9 : 0] = qspiCtrl_1__io_a5;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_qspi_qd_write = qspiCtrl_1__io_qspi_qd_write;
  assign io_qspi_qd_writeEnable = qspiCtrl_1__io_qspi_qd_writeEnable;
endmodule

module Apb3Decoder (
      input  [19:0] io_input_PADDR,
      input  [0:0] io_input_PSEL,
      input   io_input_PENABLE,
      output reg  io_input_PREADY,
      input   io_input_PWRITE,
      input  [31:0] io_input_PWDATA,
      output [31:0] io_input_PRDATA,
      output reg  io_input_PSLVERROR,
      output [19:0] io_output_PADDR,
      output reg [15:0] io_output_PSEL,
      output  io_output_PENABLE,
      input   io_output_PREADY,
      output  io_output_PWRITE,
      output [31:0] io_output_PWDATA,
      input  [31:0] io_output_PRDATA,
      input   io_output_PSLVERROR);
  wire [19:0] _zz_1_;
  wire [19:0] _zz_2_;
  wire [19:0] _zz_3_;
  wire [19:0] _zz_4_;
  wire [19:0] _zz_5_;
  wire [19:0] _zz_6_;
  wire [19:0] _zz_7_;
  wire [19:0] _zz_8_;
  wire [19:0] _zz_9_;
  wire [19:0] _zz_10_;
  wire [19:0] _zz_11_;
  wire [19:0] _zz_12_;
  wire [19:0] _zz_13_;
  wire [19:0] _zz_14_;
  wire [19:0] _zz_15_;
  wire [19:0] _zz_16_;
  assign _zz_1_ = (20'b11111111000000000000);
  assign _zz_2_ = (20'b11111111000000000000);
  assign _zz_3_ = (20'b11111111000000000000);
  assign _zz_4_ = (20'b11111111000000000000);
  assign _zz_5_ = (20'b11111111000000000000);
  assign _zz_6_ = (20'b11111111000000000000);
  assign _zz_7_ = (20'b11111111000000000000);
  assign _zz_8_ = (20'b11111111000000000000);
  assign _zz_9_ = (20'b11111111000000000000);
  assign _zz_10_ = (20'b11111111000000000000);
  assign _zz_11_ = (20'b11111111000000000000);
  assign _zz_12_ = (20'b11111111000000000000);
  assign _zz_13_ = (20'b11111111000000000000);
  assign _zz_14_ = (20'b11111111000000000000);
  assign _zz_15_ = (20'b11111111000000000000);
  assign _zz_16_ = (20'b11111111000000000000);
  assign io_output_PADDR = io_input_PADDR;
  assign io_output_PENABLE = io_input_PENABLE;
  assign io_output_PWRITE = io_input_PWRITE;
  assign io_output_PWDATA = io_input_PWDATA;
  always @ (*) begin
    io_output_PSEL[0] = (((io_input_PADDR & _zz_1_) == (20'b00000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[1] = (((io_input_PADDR & _zz_2_) == (20'b00010000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[2] = (((io_input_PADDR & _zz_3_) == (20'b11100000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[3] = (((io_input_PADDR & _zz_4_) == (20'b00100000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[4] = (((io_input_PADDR & _zz_5_) == (20'b00110000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[5] = (((io_input_PADDR & _zz_6_) == (20'b11000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[6] = (((io_input_PADDR & _zz_7_) == (20'b11010000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[7] = (((io_input_PADDR & _zz_8_) == (20'b10110000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[8] = (((io_input_PADDR & _zz_9_) == (20'b01000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[9] = (((io_input_PADDR & _zz_10_) == (20'b01010000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[10] = (((io_input_PADDR & _zz_11_) == (20'b01100000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[11] = (((io_input_PADDR & _zz_12_) == (20'b01110000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[12] = (((io_input_PADDR & _zz_13_) == (20'b10000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[13] = (((io_input_PADDR & _zz_14_) == (20'b10010000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[14] = (((io_input_PADDR & _zz_15_) == (20'b10100000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[15] = (((io_input_PADDR & _zz_16_) == (20'b11110000000000000000)) && io_input_PSEL[0]);
  end

  always @ (*) begin
    io_input_PREADY = io_output_PREADY;
    io_input_PSLVERROR = io_output_PSLVERROR;
    if((io_input_PSEL[0] && (io_output_PSEL == (16'b0000000000000000))))begin
      io_input_PREADY = 1'b1;
      io_input_PSLVERROR = 1'b1;
    end
  end

  assign io_input_PRDATA = io_output_PRDATA;
endmodule

module Apb3Router (
      input  [19:0] io_input_PADDR,
      input  [15:0] io_input_PSEL,
      input   io_input_PENABLE,
      output  io_input_PREADY,
      input   io_input_PWRITE,
      input  [31:0] io_input_PWDATA,
      output [31:0] io_input_PRDATA,
      output  io_input_PSLVERROR,
      output [19:0] io_outputs_0_PADDR,
      output [0:0] io_outputs_0_PSEL,
      output  io_outputs_0_PENABLE,
      input   io_outputs_0_PREADY,
      output  io_outputs_0_PWRITE,
      output [31:0] io_outputs_0_PWDATA,
      input  [31:0] io_outputs_0_PRDATA,
      input   io_outputs_0_PSLVERROR,
      output [19:0] io_outputs_1_PADDR,
      output [0:0] io_outputs_1_PSEL,
      output  io_outputs_1_PENABLE,
      input   io_outputs_1_PREADY,
      output  io_outputs_1_PWRITE,
      output [31:0] io_outputs_1_PWDATA,
      input  [31:0] io_outputs_1_PRDATA,
      input   io_outputs_1_PSLVERROR,
      output [19:0] io_outputs_2_PADDR,
      output [0:0] io_outputs_2_PSEL,
      output  io_outputs_2_PENABLE,
      input   io_outputs_2_PREADY,
      output  io_outputs_2_PWRITE,
      output [31:0] io_outputs_2_PWDATA,
      input  [31:0] io_outputs_2_PRDATA,
      input   io_outputs_2_PSLVERROR,
      output [19:0] io_outputs_3_PADDR,
      output [0:0] io_outputs_3_PSEL,
      output  io_outputs_3_PENABLE,
      input   io_outputs_3_PREADY,
      output  io_outputs_3_PWRITE,
      output [31:0] io_outputs_3_PWDATA,
      input  [31:0] io_outputs_3_PRDATA,
      input   io_outputs_3_PSLVERROR,
      output [19:0] io_outputs_4_PADDR,
      output [0:0] io_outputs_4_PSEL,
      output  io_outputs_4_PENABLE,
      input   io_outputs_4_PREADY,
      output  io_outputs_4_PWRITE,
      output [31:0] io_outputs_4_PWDATA,
      input  [31:0] io_outputs_4_PRDATA,
      input   io_outputs_4_PSLVERROR,
      output [19:0] io_outputs_5_PADDR,
      output [0:0] io_outputs_5_PSEL,
      output  io_outputs_5_PENABLE,
      input   io_outputs_5_PREADY,
      output  io_outputs_5_PWRITE,
      output [31:0] io_outputs_5_PWDATA,
      input  [31:0] io_outputs_5_PRDATA,
      input   io_outputs_5_PSLVERROR,
      output [19:0] io_outputs_6_PADDR,
      output [0:0] io_outputs_6_PSEL,
      output  io_outputs_6_PENABLE,
      input   io_outputs_6_PREADY,
      output  io_outputs_6_PWRITE,
      output [31:0] io_outputs_6_PWDATA,
      input  [31:0] io_outputs_6_PRDATA,
      input   io_outputs_6_PSLVERROR,
      output [19:0] io_outputs_7_PADDR,
      output [0:0] io_outputs_7_PSEL,
      output  io_outputs_7_PENABLE,
      input   io_outputs_7_PREADY,
      output  io_outputs_7_PWRITE,
      output [31:0] io_outputs_7_PWDATA,
      input  [31:0] io_outputs_7_PRDATA,
      input   io_outputs_7_PSLVERROR,
      output [19:0] io_outputs_8_PADDR,
      output [0:0] io_outputs_8_PSEL,
      output  io_outputs_8_PENABLE,
      input   io_outputs_8_PREADY,
      output  io_outputs_8_PWRITE,
      output [31:0] io_outputs_8_PWDATA,
      input  [31:0] io_outputs_8_PRDATA,
      input   io_outputs_8_PSLVERROR,
      output [19:0] io_outputs_9_PADDR,
      output [0:0] io_outputs_9_PSEL,
      output  io_outputs_9_PENABLE,
      input   io_outputs_9_PREADY,
      output  io_outputs_9_PWRITE,
      output [31:0] io_outputs_9_PWDATA,
      input  [31:0] io_outputs_9_PRDATA,
      input   io_outputs_9_PSLVERROR,
      output [19:0] io_outputs_10_PADDR,
      output [0:0] io_outputs_10_PSEL,
      output  io_outputs_10_PENABLE,
      input   io_outputs_10_PREADY,
      output  io_outputs_10_PWRITE,
      output [31:0] io_outputs_10_PWDATA,
      input  [31:0] io_outputs_10_PRDATA,
      input   io_outputs_10_PSLVERROR,
      output [19:0] io_outputs_11_PADDR,
      output [0:0] io_outputs_11_PSEL,
      output  io_outputs_11_PENABLE,
      input   io_outputs_11_PREADY,
      output  io_outputs_11_PWRITE,
      output [31:0] io_outputs_11_PWDATA,
      input  [31:0] io_outputs_11_PRDATA,
      input   io_outputs_11_PSLVERROR,
      output [19:0] io_outputs_12_PADDR,
      output [0:0] io_outputs_12_PSEL,
      output  io_outputs_12_PENABLE,
      input   io_outputs_12_PREADY,
      output  io_outputs_12_PWRITE,
      output [31:0] io_outputs_12_PWDATA,
      input  [31:0] io_outputs_12_PRDATA,
      input   io_outputs_12_PSLVERROR,
      output [19:0] io_outputs_13_PADDR,
      output [0:0] io_outputs_13_PSEL,
      output  io_outputs_13_PENABLE,
      input   io_outputs_13_PREADY,
      output  io_outputs_13_PWRITE,
      output [31:0] io_outputs_13_PWDATA,
      input  [31:0] io_outputs_13_PRDATA,
      input   io_outputs_13_PSLVERROR,
      output [19:0] io_outputs_14_PADDR,
      output [0:0] io_outputs_14_PSEL,
      output  io_outputs_14_PENABLE,
      input   io_outputs_14_PREADY,
      output  io_outputs_14_PWRITE,
      output [31:0] io_outputs_14_PWDATA,
      input  [31:0] io_outputs_14_PRDATA,
      input   io_outputs_14_PSLVERROR,
      output [19:0] io_outputs_15_PADDR,
      output [0:0] io_outputs_15_PSEL,
      output  io_outputs_15_PENABLE,
      input   io_outputs_15_PREADY,
      output  io_outputs_15_PWRITE,
      output [31:0] io_outputs_15_PWDATA,
      input  [31:0] io_outputs_15_PRDATA,
      input   io_outputs_15_PSLVERROR,
      input   toplevel_io_mainClk,
      input   toplevel_resetCtrl_systemReset);
  reg  _zz_16_;
  reg [31:0] _zz_17_;
  reg  _zz_18_;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire  _zz_8_;
  wire  _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire  _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  reg [3:0] selIndex;
  always @(*) begin
    case(selIndex)
      4'b0000 : begin
        _zz_16_ = io_outputs_0_PREADY;
        _zz_17_ = io_outputs_0_PRDATA;
        _zz_18_ = io_outputs_0_PSLVERROR;
      end
      4'b0001 : begin
        _zz_16_ = io_outputs_1_PREADY;
        _zz_17_ = io_outputs_1_PRDATA;
        _zz_18_ = io_outputs_1_PSLVERROR;
      end
      4'b0010 : begin
        _zz_16_ = io_outputs_2_PREADY;
        _zz_17_ = io_outputs_2_PRDATA;
        _zz_18_ = io_outputs_2_PSLVERROR;
      end
      4'b0011 : begin
        _zz_16_ = io_outputs_3_PREADY;
        _zz_17_ = io_outputs_3_PRDATA;
        _zz_18_ = io_outputs_3_PSLVERROR;
      end
      4'b0100 : begin
        _zz_16_ = io_outputs_4_PREADY;
        _zz_17_ = io_outputs_4_PRDATA;
        _zz_18_ = io_outputs_4_PSLVERROR;
      end
      4'b0101 : begin
        _zz_16_ = io_outputs_5_PREADY;
        _zz_17_ = io_outputs_5_PRDATA;
        _zz_18_ = io_outputs_5_PSLVERROR;
      end
      4'b0110 : begin
        _zz_16_ = io_outputs_6_PREADY;
        _zz_17_ = io_outputs_6_PRDATA;
        _zz_18_ = io_outputs_6_PSLVERROR;
      end
      4'b0111 : begin
        _zz_16_ = io_outputs_7_PREADY;
        _zz_17_ = io_outputs_7_PRDATA;
        _zz_18_ = io_outputs_7_PSLVERROR;
      end
      4'b1000 : begin
        _zz_16_ = io_outputs_8_PREADY;
        _zz_17_ = io_outputs_8_PRDATA;
        _zz_18_ = io_outputs_8_PSLVERROR;
      end
      4'b1001 : begin
        _zz_16_ = io_outputs_9_PREADY;
        _zz_17_ = io_outputs_9_PRDATA;
        _zz_18_ = io_outputs_9_PSLVERROR;
      end
      4'b1010 : begin
        _zz_16_ = io_outputs_10_PREADY;
        _zz_17_ = io_outputs_10_PRDATA;
        _zz_18_ = io_outputs_10_PSLVERROR;
      end
      4'b1011 : begin
        _zz_16_ = io_outputs_11_PREADY;
        _zz_17_ = io_outputs_11_PRDATA;
        _zz_18_ = io_outputs_11_PSLVERROR;
      end
      4'b1100 : begin
        _zz_16_ = io_outputs_12_PREADY;
        _zz_17_ = io_outputs_12_PRDATA;
        _zz_18_ = io_outputs_12_PSLVERROR;
      end
      4'b1101 : begin
        _zz_16_ = io_outputs_13_PREADY;
        _zz_17_ = io_outputs_13_PRDATA;
        _zz_18_ = io_outputs_13_PSLVERROR;
      end
      4'b1110 : begin
        _zz_16_ = io_outputs_14_PREADY;
        _zz_17_ = io_outputs_14_PRDATA;
        _zz_18_ = io_outputs_14_PSLVERROR;
      end
      default : begin
        _zz_16_ = io_outputs_15_PREADY;
        _zz_17_ = io_outputs_15_PRDATA;
        _zz_18_ = io_outputs_15_PSLVERROR;
      end
    endcase
  end

  assign io_outputs_0_PADDR = io_input_PADDR;
  assign io_outputs_0_PENABLE = io_input_PENABLE;
  assign io_outputs_0_PSEL[0] = io_input_PSEL[0];
  assign io_outputs_0_PWRITE = io_input_PWRITE;
  assign io_outputs_0_PWDATA = io_input_PWDATA;
  assign io_outputs_1_PADDR = io_input_PADDR;
  assign io_outputs_1_PENABLE = io_input_PENABLE;
  assign io_outputs_1_PSEL[0] = io_input_PSEL[1];
  assign io_outputs_1_PWRITE = io_input_PWRITE;
  assign io_outputs_1_PWDATA = io_input_PWDATA;
  assign io_outputs_2_PADDR = io_input_PADDR;
  assign io_outputs_2_PENABLE = io_input_PENABLE;
  assign io_outputs_2_PSEL[0] = io_input_PSEL[2];
  assign io_outputs_2_PWRITE = io_input_PWRITE;
  assign io_outputs_2_PWDATA = io_input_PWDATA;
  assign io_outputs_3_PADDR = io_input_PADDR;
  assign io_outputs_3_PENABLE = io_input_PENABLE;
  assign io_outputs_3_PSEL[0] = io_input_PSEL[3];
  assign io_outputs_3_PWRITE = io_input_PWRITE;
  assign io_outputs_3_PWDATA = io_input_PWDATA;
  assign io_outputs_4_PADDR = io_input_PADDR;
  assign io_outputs_4_PENABLE = io_input_PENABLE;
  assign io_outputs_4_PSEL[0] = io_input_PSEL[4];
  assign io_outputs_4_PWRITE = io_input_PWRITE;
  assign io_outputs_4_PWDATA = io_input_PWDATA;
  assign io_outputs_5_PADDR = io_input_PADDR;
  assign io_outputs_5_PENABLE = io_input_PENABLE;
  assign io_outputs_5_PSEL[0] = io_input_PSEL[5];
  assign io_outputs_5_PWRITE = io_input_PWRITE;
  assign io_outputs_5_PWDATA = io_input_PWDATA;
  assign io_outputs_6_PADDR = io_input_PADDR;
  assign io_outputs_6_PENABLE = io_input_PENABLE;
  assign io_outputs_6_PSEL[0] = io_input_PSEL[6];
  assign io_outputs_6_PWRITE = io_input_PWRITE;
  assign io_outputs_6_PWDATA = io_input_PWDATA;
  assign io_outputs_7_PADDR = io_input_PADDR;
  assign io_outputs_7_PENABLE = io_input_PENABLE;
  assign io_outputs_7_PSEL[0] = io_input_PSEL[7];
  assign io_outputs_7_PWRITE = io_input_PWRITE;
  assign io_outputs_7_PWDATA = io_input_PWDATA;
  assign io_outputs_8_PADDR = io_input_PADDR;
  assign io_outputs_8_PENABLE = io_input_PENABLE;
  assign io_outputs_8_PSEL[0] = io_input_PSEL[8];
  assign io_outputs_8_PWRITE = io_input_PWRITE;
  assign io_outputs_8_PWDATA = io_input_PWDATA;
  assign io_outputs_9_PADDR = io_input_PADDR;
  assign io_outputs_9_PENABLE = io_input_PENABLE;
  assign io_outputs_9_PSEL[0] = io_input_PSEL[9];
  assign io_outputs_9_PWRITE = io_input_PWRITE;
  assign io_outputs_9_PWDATA = io_input_PWDATA;
  assign io_outputs_10_PADDR = io_input_PADDR;
  assign io_outputs_10_PENABLE = io_input_PENABLE;
  assign io_outputs_10_PSEL[0] = io_input_PSEL[10];
  assign io_outputs_10_PWRITE = io_input_PWRITE;
  assign io_outputs_10_PWDATA = io_input_PWDATA;
  assign io_outputs_11_PADDR = io_input_PADDR;
  assign io_outputs_11_PENABLE = io_input_PENABLE;
  assign io_outputs_11_PSEL[0] = io_input_PSEL[11];
  assign io_outputs_11_PWRITE = io_input_PWRITE;
  assign io_outputs_11_PWDATA = io_input_PWDATA;
  assign io_outputs_12_PADDR = io_input_PADDR;
  assign io_outputs_12_PENABLE = io_input_PENABLE;
  assign io_outputs_12_PSEL[0] = io_input_PSEL[12];
  assign io_outputs_12_PWRITE = io_input_PWRITE;
  assign io_outputs_12_PWDATA = io_input_PWDATA;
  assign io_outputs_13_PADDR = io_input_PADDR;
  assign io_outputs_13_PENABLE = io_input_PENABLE;
  assign io_outputs_13_PSEL[0] = io_input_PSEL[13];
  assign io_outputs_13_PWRITE = io_input_PWRITE;
  assign io_outputs_13_PWDATA = io_input_PWDATA;
  assign io_outputs_14_PADDR = io_input_PADDR;
  assign io_outputs_14_PENABLE = io_input_PENABLE;
  assign io_outputs_14_PSEL[0] = io_input_PSEL[14];
  assign io_outputs_14_PWRITE = io_input_PWRITE;
  assign io_outputs_14_PWDATA = io_input_PWDATA;
  assign io_outputs_15_PADDR = io_input_PADDR;
  assign io_outputs_15_PENABLE = io_input_PENABLE;
  assign io_outputs_15_PSEL[0] = io_input_PSEL[15];
  assign io_outputs_15_PWRITE = io_input_PWRITE;
  assign io_outputs_15_PWDATA = io_input_PWDATA;
  assign _zz_1_ = io_input_PSEL[3];
  assign _zz_2_ = io_input_PSEL[5];
  assign _zz_3_ = io_input_PSEL[6];
  assign _zz_4_ = io_input_PSEL[7];
  assign _zz_5_ = io_input_PSEL[9];
  assign _zz_6_ = io_input_PSEL[10];
  assign _zz_7_ = io_input_PSEL[11];
  assign _zz_8_ = io_input_PSEL[12];
  assign _zz_9_ = io_input_PSEL[13];
  assign _zz_10_ = io_input_PSEL[14];
  assign _zz_11_ = io_input_PSEL[15];
  assign _zz_12_ = (((((((io_input_PSEL[1] || _zz_1_) || _zz_2_) || _zz_4_) || _zz_5_) || _zz_7_) || _zz_9_) || _zz_11_);
  assign _zz_13_ = (((((((io_input_PSEL[2] || _zz_1_) || _zz_3_) || _zz_4_) || _zz_6_) || _zz_7_) || _zz_10_) || _zz_11_);
  assign _zz_14_ = (((((((io_input_PSEL[4] || _zz_2_) || _zz_3_) || _zz_4_) || _zz_8_) || _zz_9_) || _zz_10_) || _zz_11_);
  assign _zz_15_ = (((((((io_input_PSEL[8] || _zz_5_) || _zz_6_) || _zz_7_) || _zz_8_) || _zz_9_) || _zz_10_) || _zz_11_);
  assign io_input_PREADY = _zz_16_;
  assign io_input_PRDATA = _zz_17_;
  assign io_input_PSLVERROR = _zz_18_;
  always @ (posedge toplevel_io_mainClk) begin
    selIndex <= {_zz_15_,{_zz_14_,{_zz_13_,_zz_12_}}};
  end

endmodule

module MuraxArduino (
      input   io_asyncReset,
      input   io_mainClk,
      input   io_jtag_tms,
      input   io_jtag_tdi,
      output  io_jtag_tdo,
      input   io_jtag_tck,
      input  [31:0] io_gpioA_read,
      output [31:0] io_gpioA_write,
      output [31:0] io_gpioA_writeEnable,
      output  io_uart_txd,
      input   io_uart_rxd,
      input  [1:0] io_pinInterrupt_pins,
      output [2:0] io_pwm_pins,
      output  io_servo_pin,
      output [31:0] io_mux_pins,
      output  io_tone_pin,
      output  io_shiftOut_dataPin,
      output  io_shiftOut_clockPin,
      output [0:0] io_spiMaster_ss,
      output  io_spiMaster_sclk,
      output  io_spiMaster_mosi,
      input   io_spiMaster_miso,
      output  io_i2c_sda_write,
      input   io_i2c_sda_read,
      output  io_i2c_scl_write,
      input   io_i2c_scl_read,
      input   io_pulseIn_pin,
      output  io_sevenSegment_digitPin,
      output [6:0] io_sevenSegment_segPins,
      input   io_shiftIn_dataPin,
      output  io_shiftIn_clockPin,
      input   io_qspi_qck,
      input   io_qspi_qss,
      input  [3:0] io_qspi_qd_read,
      output [3:0] io_qspi_qd_write,
      output [3:0] io_qspi_qd_writeEnable,
      output [17:0] io_sram_addr,
      input  [15:0] io_sram_dat_read,
      output [15:0] io_sram_dat_write,
      output  io_sram_dat_writeEnable,
      output  io_sram_cs,
      output  io_sram_we,
      output  io_sram_oe,
      output  io_sram_lb,
      output  io_sram_ub);
  wire  _zz_11_;
  wire [7:0] _zz_12_;
  reg  _zz_13_;
  reg  _zz_14_;
  reg  _zz_15_;
  wire [3:0] _zz_16_;
  wire [3:0] _zz_17_;
  wire [7:0] _zz_18_;
  wire [7:0] _zz_19_;
  wire [7:0] _zz_20_;
  wire [7:0] _zz_21_;
  wire [7:0] _zz_22_;
  wire [7:0] _zz_23_;
  wire [7:0] _zz_24_;
  wire [7:0] _zz_25_;
  wire [7:0] _zz_26_;
  wire [7:0] _zz_27_;
  wire [7:0] _zz_28_;
  wire [7:0] _zz_29_;
  wire [7:0] _zz_30_;
  wire [7:0] _zz_31_;
  wire  _zz_32_;
  wire  _zz_33_;
  wire  _zz_34_;
  reg [31:0] _zz_35_;
  wire  bufferCC_5__io_dataOut;
  wire  system_mainBusArbiter_io_iBus_cmd_ready;
  wire  system_mainBusArbiter_io_iBus_rsp_valid;
  wire  system_mainBusArbiter_io_iBus_rsp_payload_error;
  wire [31:0] system_mainBusArbiter_io_iBus_rsp_payload_inst;
  wire  system_mainBusArbiter_io_dBus_cmd_ready;
  wire  system_mainBusArbiter_io_dBus_rsp_ready;
  wire  system_mainBusArbiter_io_dBus_rsp_error;
  wire [31:0] system_mainBusArbiter_io_dBus_rsp_data;
  wire  system_mainBusArbiter_io_masterBus_cmd_valid;
  wire  system_mainBusArbiter_io_masterBus_cmd_payload_write;
  wire [31:0] system_mainBusArbiter_io_masterBus_cmd_payload_address;
  wire [31:0] system_mainBusArbiter_io_masterBus_cmd_payload_data;
  wire [3:0] system_mainBusArbiter_io_masterBus_cmd_payload_mask;
  wire  system_cpu_iBus_cmd_valid;
  wire [31:0] system_cpu_iBus_cmd_payload_pc;
  wire  system_cpu_debug_bus_cmd_ready;
  wire [31:0] system_cpu_debug_bus_rsp_data;
  wire  system_cpu_debug_resetOut;
  wire  system_cpu_dBus_cmd_valid;
  wire  system_cpu_dBus_cmd_payload_wr;
  wire [31:0] system_cpu_dBus_cmd_payload_address;
  wire [31:0] system_cpu_dBus_cmd_payload_data;
  wire [1:0] system_cpu_dBus_cmd_payload_size;
  wire  jtagBridge_1__io_jtag_tdo;
  wire  jtagBridge_1__io_remote_cmd_valid;
  wire  jtagBridge_1__io_remote_cmd_payload_last;
  wire [0:0] jtagBridge_1__io_remote_cmd_payload_fragment;
  wire  jtagBridge_1__io_remote_rsp_ready;
  wire  systemDebugger_1__io_remote_cmd_ready;
  wire  systemDebugger_1__io_remote_rsp_valid;
  wire  systemDebugger_1__io_remote_rsp_payload_error;
  wire [31:0] systemDebugger_1__io_remote_rsp_payload_data;
  wire  systemDebugger_1__io_mem_cmd_valid;
  wire [31:0] systemDebugger_1__io_mem_cmd_payload_address;
  wire [31:0] systemDebugger_1__io_mem_cmd_payload_data;
  wire  systemDebugger_1__io_mem_cmd_payload_wr;
  wire [1:0] systemDebugger_1__io_mem_cmd_payload_size;
  wire  system_ram_io_bus_cmd_ready;
  wire  system_ram_io_bus_rsp_valid;
  wire [31:0] system_ram_io_bus_rsp_0_data;
  wire  system_sramCtrl_io_bus_cmd_ready;
  wire  system_sramCtrl_io_bus_rsp_valid;
  wire [31:0] system_sramCtrl_io_bus_rsp_1_data;
  wire [17:0] system_sramCtrl_io_sram_addr;
  wire  system_sramCtrl_io_sram_cs;
  wire  system_sramCtrl_io_sram_we;
  wire  system_sramCtrl_io_sram_oe;
  wire  system_sramCtrl_io_sram_lb;
  wire  system_sramCtrl_io_sram_ub;
  wire [15:0] system_sramCtrl_io_sram_dat_write;
  wire  system_sramCtrl_io_sram_dat_writeEnable;
  wire  system_apbBridge_io_pipelinedMemoryBus_cmd_ready;
  wire  system_apbBridge_io_pipelinedMemoryBus_rsp_valid;
  wire [31:0] system_apbBridge_io_pipelinedMemoryBus_rsp_2_data;
  wire [19:0] system_apbBridge_io_apb_PADDR;
  wire [0:0] system_apbBridge_io_apb_PSEL;
  wire  system_apbBridge_io_apb_PENABLE;
  wire  system_apbBridge_io_apb_PWRITE;
  wire [31:0] system_apbBridge_io_apb_PWDATA;
  wire  system_gpioACtrl_io_apb_PREADY;
  wire [31:0] system_gpioACtrl_io_apb_PRDATA;
  wire  system_gpioACtrl_io_apb_PSLVERROR;
  wire [31:0] system_gpioACtrl_io_gpio_write;
  wire [31:0] system_gpioACtrl_io_gpio_writeEnable;
  wire  system_uartCtrl_io_apb_PREADY;
  wire [31:0] system_uartCtrl_io_apb_PRDATA;
  wire  system_uartCtrl_io_uart_txd;
  wire  system_uartCtrl_io_interrupt;
  wire  system_pinInterruptCtrl_io_apb_PREADY;
  wire [31:0] system_pinInterruptCtrl_io_apb_PRDATA;
  wire  system_pinInterruptCtrl_io_apb_PSLVERROR;
  wire  system_pinInterruptCtrl_io_interrupt;
  wire  system_timer_io_apb_PREADY;
  wire [31:0] system_timer_io_apb_PRDATA;
  wire  system_timer_io_apb_PSLVERROR;
  wire  system_timer_io_interrupt;
  wire  system_pwmCtrl_io_apb_PREADY;
  wire [31:0] system_pwmCtrl_io_apb_PRDATA;
  wire  system_pwmCtrl_io_apb_PSLVERROR;
  wire [2:0] system_pwmCtrl_io_pwm_pins;
  wire  system_servoCtrl_io_apb_PREADY;
  wire [31:0] system_servoCtrl_io_apb_PRDATA;
  wire  system_servoCtrl_io_apb_PSLVERROR;
  wire  system_servoCtrl_io_servo_pin;
  wire  system_muxCtrl_io_apb_PREADY;
  wire [31:0] system_muxCtrl_io_apb_PRDATA;
  wire  system_muxCtrl_io_apb_PSLVERROR;
  wire [31:0] system_muxCtrl_io_mux_pins;
  wire  system_machineTimerCtrl_io_apb_PREADY;
  wire [31:0] system_machineTimerCtrl_io_apb_PRDATA;
  wire  system_machineTimerCtrl_io_apb_PSLVERROR;
  wire  system_toneCtrl_io_apb_PREADY;
  wire [31:0] system_toneCtrl_io_apb_PRDATA;
  wire  system_toneCtrl_io_apb_PSLVERROR;
  wire  system_toneCtrl_io_tone_pin;
  wire  system_shiftOutCtrl_io_apb_PREADY;
  wire [31:0] system_shiftOutCtrl_io_apb_PRDATA;
  wire  system_shiftOutCtrl_io_apb_PSLVERROR;
  wire  system_shiftOutCtrl_io_shiftOut_dataPin;
  wire  system_shiftOutCtrl_io_shiftOut_clockPin;
  wire  system_spiMasterCtrl_io_apb_PREADY;
  wire [31:0] system_spiMasterCtrl_io_apb_PRDATA;
  wire  system_spiMasterCtrl_io_spi_sclk;
  wire  system_spiMasterCtrl_io_spi_mosi;
  wire [0:0] system_spiMasterCtrl_io_spi_ss;
  wire  system_spiMasterCtrl_io_interrupt;
  wire  system_i2cCtrl_io_apb_PREADY;
  wire [31:0] system_i2cCtrl_io_apb_PRDATA;
  wire  system_i2cCtrl_io_i2c_scl_write;
  wire  system_i2cCtrl_io_i2c_sda_write;
  wire  system_i2cCtrl_io_interrupt;
  wire  system_pulseInCtrl_io_apb_PREADY;
  wire [31:0] system_pulseInCtrl_io_apb_PRDATA;
  wire  system_pulseInCtrl_io_apb_PSLVERROR;
  wire  system_sevenSegmentCtrl_io_apb_PREADY;
  wire [31:0] system_sevenSegmentCtrl_io_apb_PRDATA;
  wire  system_sevenSegmentCtrl_io_apb_PSLVERROR;
  wire  system_sevenSegmentCtrl_io_sevenSegment_digitPin;
  wire [6:0] system_sevenSegmentCtrl_io_sevenSegment_segPins;
  wire  system_shiftInCtrl_io_apb_PREADY;
  wire [31:0] system_shiftInCtrl_io_apb_PRDATA;
  wire  system_shiftInCtrl_io_apb_PSLVERROR;
  wire  system_shiftInCtrl_io_shiftIn_clockPin;
  wire  system_qspiCtrl_io_apb_PREADY;
  wire [31:0] system_qspiCtrl_io_apb_PRDATA;
  wire  system_qspiCtrl_io_apb_PSLVERROR;
  wire [3:0] system_qspiCtrl_io_qspi_qd_write;
  wire [3:0] system_qspiCtrl_io_qspi_qd_writeEnable;
  wire  io_apb_decoder_io_input_PREADY;
  wire [31:0] io_apb_decoder_io_input_PRDATA;
  wire  io_apb_decoder_io_input_PSLVERROR;
  wire [19:0] io_apb_decoder_io_output_PADDR;
  wire [15:0] io_apb_decoder_io_output_PSEL;
  wire  io_apb_decoder_io_output_PENABLE;
  wire  io_apb_decoder_io_output_PWRITE;
  wire [31:0] io_apb_decoder_io_output_PWDATA;
  wire  apb3Router_1__io_input_PREADY;
  wire [31:0] apb3Router_1__io_input_PRDATA;
  wire  apb3Router_1__io_input_PSLVERROR;
  wire [19:0] apb3Router_1__io_outputs_0_PADDR;
  wire [0:0] apb3Router_1__io_outputs_0_PSEL;
  wire  apb3Router_1__io_outputs_0_PENABLE;
  wire  apb3Router_1__io_outputs_0_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_0_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_1_PADDR;
  wire [0:0] apb3Router_1__io_outputs_1_PSEL;
  wire  apb3Router_1__io_outputs_1_PENABLE;
  wire  apb3Router_1__io_outputs_1_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_1_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_2_PADDR;
  wire [0:0] apb3Router_1__io_outputs_2_PSEL;
  wire  apb3Router_1__io_outputs_2_PENABLE;
  wire  apb3Router_1__io_outputs_2_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_2_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_3_PADDR;
  wire [0:0] apb3Router_1__io_outputs_3_PSEL;
  wire  apb3Router_1__io_outputs_3_PENABLE;
  wire  apb3Router_1__io_outputs_3_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_3_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_4_PADDR;
  wire [0:0] apb3Router_1__io_outputs_4_PSEL;
  wire  apb3Router_1__io_outputs_4_PENABLE;
  wire  apb3Router_1__io_outputs_4_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_4_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_5_PADDR;
  wire [0:0] apb3Router_1__io_outputs_5_PSEL;
  wire  apb3Router_1__io_outputs_5_PENABLE;
  wire  apb3Router_1__io_outputs_5_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_5_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_6_PADDR;
  wire [0:0] apb3Router_1__io_outputs_6_PSEL;
  wire  apb3Router_1__io_outputs_6_PENABLE;
  wire  apb3Router_1__io_outputs_6_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_6_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_7_PADDR;
  wire [0:0] apb3Router_1__io_outputs_7_PSEL;
  wire  apb3Router_1__io_outputs_7_PENABLE;
  wire  apb3Router_1__io_outputs_7_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_7_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_8_PADDR;
  wire [0:0] apb3Router_1__io_outputs_8_PSEL;
  wire  apb3Router_1__io_outputs_8_PENABLE;
  wire  apb3Router_1__io_outputs_8_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_8_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_9_PADDR;
  wire [0:0] apb3Router_1__io_outputs_9_PSEL;
  wire  apb3Router_1__io_outputs_9_PENABLE;
  wire  apb3Router_1__io_outputs_9_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_9_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_10_PADDR;
  wire [0:0] apb3Router_1__io_outputs_10_PSEL;
  wire  apb3Router_1__io_outputs_10_PENABLE;
  wire  apb3Router_1__io_outputs_10_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_10_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_11_PADDR;
  wire [0:0] apb3Router_1__io_outputs_11_PSEL;
  wire  apb3Router_1__io_outputs_11_PENABLE;
  wire  apb3Router_1__io_outputs_11_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_11_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_12_PADDR;
  wire [0:0] apb3Router_1__io_outputs_12_PSEL;
  wire  apb3Router_1__io_outputs_12_PENABLE;
  wire  apb3Router_1__io_outputs_12_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_12_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_13_PADDR;
  wire [0:0] apb3Router_1__io_outputs_13_PSEL;
  wire  apb3Router_1__io_outputs_13_PENABLE;
  wire  apb3Router_1__io_outputs_13_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_13_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_14_PADDR;
  wire [0:0] apb3Router_1__io_outputs_14_PSEL;
  wire  apb3Router_1__io_outputs_14_PENABLE;
  wire  apb3Router_1__io_outputs_14_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_14_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_15_PADDR;
  wire [0:0] apb3Router_1__io_outputs_15_PSEL;
  wire  apb3Router_1__io_outputs_15_PENABLE;
  wire  apb3Router_1__io_outputs_15_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_15_PWDATA;
  wire  _zz_36_;
  wire  _zz_37_;
  wire  _zz_38_;
  wire [31:0] _zz_39_;
  wire [31:0] _zz_40_;
  reg  resetCtrl_mainClkResetUnbuffered;
  reg [5:0] resetCtrl_systemClkResetCounter = (6'b000000);
  wire [5:0] _zz_1_;
  reg  resetCtrl_mainClkReset;
  reg  resetCtrl_systemReset;
  reg  system_timerInterrupt;
  reg  system_externalInterrupt;
  wire  system_cpu_dBus_cmd_halfPipe_valid;
  wire  system_cpu_dBus_cmd_halfPipe_ready;
  wire  system_cpu_dBus_cmd_halfPipe_payload_wr;
  wire [31:0] system_cpu_dBus_cmd_halfPipe_payload_address;
  wire [31:0] system_cpu_dBus_cmd_halfPipe_payload_data;
  wire [1:0] system_cpu_dBus_cmd_halfPipe_payload_size;
  reg  system_cpu_dBus_cmd_halfPipe_regs_valid;
  reg  system_cpu_dBus_cmd_halfPipe_regs_ready;
  reg  system_cpu_dBus_cmd_halfPipe_regs_payload_wr;
  reg [31:0] system_cpu_dBus_cmd_halfPipe_regs_payload_address;
  reg [31:0] system_cpu_dBus_cmd_halfPipe_regs_payload_data;
  reg [1:0] system_cpu_dBus_cmd_halfPipe_regs_payload_size;
  reg  system_cpu_debug_resetOut_regNext;
  reg  _zz_2_;
  wire  system_mainBusArbiter_io_masterBus_cmd_s2mPipe_valid;
  wire  system_mainBusArbiter_io_masterBus_cmd_s2mPipe_ready;
  wire  system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_write;
  wire [31:0] system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_address;
  wire [31:0] system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_data;
  wire [3:0] system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_mask;
  reg  _zz_3_;
  reg  _zz_4_;
  reg [31:0] _zz_5_;
  reg [31:0] _zz_6_;
  reg [3:0] _zz_7_;
  wire  system_mainBusDecoder_logic_masterPipelined_cmd_valid;
  reg  system_mainBusDecoder_logic_masterPipelined_cmd_ready;
  wire  system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  wire [31:0] system_mainBusDecoder_logic_masterPipelined_cmd_payload_address;
  wire [31:0] system_mainBusDecoder_logic_masterPipelined_cmd_payload_data;
  wire [3:0] system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask;
  wire  system_mainBusDecoder_logic_masterPipelined_rsp_valid;
  wire [31:0] system_mainBusDecoder_logic_masterPipelined_rsp_payload_data;
  wire  system_mainBusDecoder_logic_hits_0;
  wire  _zz_8_;
  wire  system_mainBusDecoder_logic_hits_1;
  wire  _zz_9_;
  wire  system_mainBusDecoder_logic_hits_2;
  wire  _zz_10_;
  wire  system_mainBusDecoder_logic_noHit;
  reg  system_mainBusDecoder_logic_rspPending;
  reg  system_mainBusDecoder_logic_rspNoHit;
  reg [1:0] system_mainBusDecoder_logic_rspSourceId;
  assign _zz_36_ = (resetCtrl_systemClkResetCounter != _zz_1_);
  assign _zz_37_ = (! system_cpu_dBus_cmd_halfPipe_regs_valid);
  assign _zz_38_ = (_zz_11_ && (! system_mainBusArbiter_io_masterBus_cmd_s2mPipe_ready));
  assign _zz_39_ = (32'b11111111111110000000000000000000);
  assign _zz_40_ = (32'b11111111111100000000000000000000);
  BufferCC_4_ bufferCC_5_ ( 
    .io_dataIn(io_asyncReset),
    .io_dataOut(bufferCC_5__io_dataOut),
    .toplevel_io_mainClk(io_mainClk) 
  );
  MuraxMasterArbiter system_mainBusArbiter ( 
    .io_iBus_cmd_valid(system_cpu_iBus_cmd_valid),
    .io_iBus_cmd_ready(system_mainBusArbiter_io_iBus_cmd_ready),
    .io_iBus_cmd_payload_pc(system_cpu_iBus_cmd_payload_pc),
    .io_iBus_rsp_valid(system_mainBusArbiter_io_iBus_rsp_valid),
    .io_iBus_rsp_payload_error(system_mainBusArbiter_io_iBus_rsp_payload_error),
    .io_iBus_rsp_payload_inst(system_mainBusArbiter_io_iBus_rsp_payload_inst),
    .io_dBus_cmd_valid(system_cpu_dBus_cmd_halfPipe_valid),
    .io_dBus_cmd_ready(system_mainBusArbiter_io_dBus_cmd_ready),
    .io_dBus_cmd_payload_wr(system_cpu_dBus_cmd_halfPipe_payload_wr),
    .io_dBus_cmd_payload_address(system_cpu_dBus_cmd_halfPipe_payload_address),
    .io_dBus_cmd_payload_data(system_cpu_dBus_cmd_halfPipe_payload_data),
    .io_dBus_cmd_payload_size(system_cpu_dBus_cmd_halfPipe_payload_size),
    .io_dBus_rsp_ready(system_mainBusArbiter_io_dBus_rsp_ready),
    .io_dBus_rsp_error(system_mainBusArbiter_io_dBus_rsp_error),
    .io_dBus_rsp_data(system_mainBusArbiter_io_dBus_rsp_data),
    .io_masterBus_cmd_valid(system_mainBusArbiter_io_masterBus_cmd_valid),
    .io_masterBus_cmd_ready(_zz_11_),
    .io_masterBus_cmd_payload_write(system_mainBusArbiter_io_masterBus_cmd_payload_write),
    .io_masterBus_cmd_payload_address(system_mainBusArbiter_io_masterBus_cmd_payload_address),
    .io_masterBus_cmd_payload_data(system_mainBusArbiter_io_masterBus_cmd_payload_data),
    .io_masterBus_cmd_payload_mask(system_mainBusArbiter_io_masterBus_cmd_payload_mask),
    .io_masterBus_rsp_valid(system_mainBusDecoder_logic_masterPipelined_rsp_valid),
    .io_masterBus_rsp_payload_data(system_mainBusDecoder_logic_masterPipelined_rsp_payload_data),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  VexRiscv system_cpu ( 
    .iBus_cmd_valid(system_cpu_iBus_cmd_valid),
    .iBus_cmd_ready(system_mainBusArbiter_io_iBus_cmd_ready),
    .iBus_cmd_payload_pc(system_cpu_iBus_cmd_payload_pc),
    .iBus_rsp_valid(system_mainBusArbiter_io_iBus_rsp_valid),
    .iBus_rsp_payload_error(system_mainBusArbiter_io_iBus_rsp_payload_error),
    .iBus_rsp_payload_inst(system_mainBusArbiter_io_iBus_rsp_payload_inst),
    .timerInterrupt(system_timerInterrupt),
    .externalInterrupt(system_externalInterrupt),
    .debug_bus_cmd_valid(systemDebugger_1__io_mem_cmd_valid),
    .debug_bus_cmd_ready(system_cpu_debug_bus_cmd_ready),
    .debug_bus_cmd_payload_wr(systemDebugger_1__io_mem_cmd_payload_wr),
    .debug_bus_cmd_payload_address(_zz_12_),
    .debug_bus_cmd_payload_data(systemDebugger_1__io_mem_cmd_payload_data),
    .debug_bus_rsp_data(system_cpu_debug_bus_rsp_data),
    .debug_resetOut(system_cpu_debug_resetOut),
    .dBus_cmd_valid(system_cpu_dBus_cmd_valid),
    .dBus_cmd_ready(system_cpu_dBus_cmd_halfPipe_regs_ready),
    .dBus_cmd_payload_wr(system_cpu_dBus_cmd_payload_wr),
    .dBus_cmd_payload_address(system_cpu_dBus_cmd_payload_address),
    .dBus_cmd_payload_data(system_cpu_dBus_cmd_payload_data),
    .dBus_cmd_payload_size(system_cpu_dBus_cmd_payload_size),
    .dBus_rsp_ready(system_mainBusArbiter_io_dBus_rsp_ready),
    .dBus_rsp_error(system_mainBusArbiter_io_dBus_rsp_error),
    .dBus_rsp_data(system_mainBusArbiter_io_dBus_rsp_data),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset),
    .toplevel_resetCtrl_mainClkReset(resetCtrl_mainClkReset) 
  );
  JtagBridge jtagBridge_1_ ( 
    .io_jtag_tms(io_jtag_tms),
    .io_jtag_tdi(io_jtag_tdi),
    .io_jtag_tdo(jtagBridge_1__io_jtag_tdo),
    .io_jtag_tck(io_jtag_tck),
    .io_remote_cmd_valid(jtagBridge_1__io_remote_cmd_valid),
    .io_remote_cmd_ready(systemDebugger_1__io_remote_cmd_ready),
    .io_remote_cmd_payload_last(jtagBridge_1__io_remote_cmd_payload_last),
    .io_remote_cmd_payload_fragment(jtagBridge_1__io_remote_cmd_payload_fragment),
    .io_remote_rsp_valid(systemDebugger_1__io_remote_rsp_valid),
    .io_remote_rsp_ready(jtagBridge_1__io_remote_rsp_ready),
    .io_remote_rsp_payload_error(systemDebugger_1__io_remote_rsp_payload_error),
    .io_remote_rsp_payload_data(systemDebugger_1__io_remote_rsp_payload_data),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_mainClkReset(resetCtrl_mainClkReset) 
  );
  SystemDebugger systemDebugger_1_ ( 
    .io_remote_cmd_valid(jtagBridge_1__io_remote_cmd_valid),
    .io_remote_cmd_ready(systemDebugger_1__io_remote_cmd_ready),
    .io_remote_cmd_payload_last(jtagBridge_1__io_remote_cmd_payload_last),
    .io_remote_cmd_payload_fragment(jtagBridge_1__io_remote_cmd_payload_fragment),
    .io_remote_rsp_valid(systemDebugger_1__io_remote_rsp_valid),
    .io_remote_rsp_ready(jtagBridge_1__io_remote_rsp_ready),
    .io_remote_rsp_payload_error(systemDebugger_1__io_remote_rsp_payload_error),
    .io_remote_rsp_payload_data(systemDebugger_1__io_remote_rsp_payload_data),
    .io_mem_cmd_valid(systemDebugger_1__io_mem_cmd_valid),
    .io_mem_cmd_ready(system_cpu_debug_bus_cmd_ready),
    .io_mem_cmd_payload_address(systemDebugger_1__io_mem_cmd_payload_address),
    .io_mem_cmd_payload_data(systemDebugger_1__io_mem_cmd_payload_data),
    .io_mem_cmd_payload_wr(systemDebugger_1__io_mem_cmd_payload_wr),
    .io_mem_cmd_payload_size(systemDebugger_1__io_mem_cmd_payload_size),
    .io_mem_rsp_valid(_zz_2_),
    .io_mem_rsp_payload(system_cpu_debug_bus_rsp_data),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_mainClkReset(resetCtrl_mainClkReset) 
  );
  MuraxPipelinedMemoryBusRam system_ram ( 
    .io_bus_cmd_valid(_zz_13_),
    .io_bus_cmd_ready(system_ram_io_bus_cmd_ready),
    .io_bus_cmd_payload_write(_zz_8_),
    .io_bus_cmd_payload_address(system_mainBusDecoder_logic_masterPipelined_cmd_payload_address),
    .io_bus_cmd_payload_data(system_mainBusDecoder_logic_masterPipelined_cmd_payload_data),
    .io_bus_cmd_payload_mask(system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask),
    .io_bus_rsp_valid(system_ram_io_bus_rsp_valid),
    .io_bus_rsp_0_data(system_ram_io_bus_rsp_0_data),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  MuraxPipelinedMemoryBusSram system_sramCtrl ( 
    .io_bus_cmd_valid(_zz_14_),
    .io_bus_cmd_ready(system_sramCtrl_io_bus_cmd_ready),
    .io_bus_cmd_payload_write(_zz_9_),
    .io_bus_cmd_payload_address(system_mainBusDecoder_logic_masterPipelined_cmd_payload_address),
    .io_bus_cmd_payload_data(system_mainBusDecoder_logic_masterPipelined_cmd_payload_data),
    .io_bus_cmd_payload_mask(system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask),
    .io_bus_rsp_valid(system_sramCtrl_io_bus_rsp_valid),
    .io_bus_rsp_1_data(system_sramCtrl_io_bus_rsp_1_data),
    .io_sram_addr(system_sramCtrl_io_sram_addr),
    .io_sram_dat_read(io_sram_dat_read),
    .io_sram_dat_write(system_sramCtrl_io_sram_dat_write),
    .io_sram_dat_writeEnable(system_sramCtrl_io_sram_dat_writeEnable),
    .io_sram_cs(system_sramCtrl_io_sram_cs),
    .io_sram_we(system_sramCtrl_io_sram_we),
    .io_sram_oe(system_sramCtrl_io_sram_oe),
    .io_sram_lb(system_sramCtrl_io_sram_lb),
    .io_sram_ub(system_sramCtrl_io_sram_ub),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  PipelinedMemoryBusToApbBridge system_apbBridge ( 
    .io_pipelinedMemoryBus_cmd_valid(_zz_15_),
    .io_pipelinedMemoryBus_cmd_ready(system_apbBridge_io_pipelinedMemoryBus_cmd_ready),
    .io_pipelinedMemoryBus_cmd_payload_write(_zz_10_),
    .io_pipelinedMemoryBus_cmd_payload_address(system_mainBusDecoder_logic_masterPipelined_cmd_payload_address),
    .io_pipelinedMemoryBus_cmd_payload_data(system_mainBusDecoder_logic_masterPipelined_cmd_payload_data),
    .io_pipelinedMemoryBus_cmd_payload_mask(system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask),
    .io_pipelinedMemoryBus_rsp_valid(system_apbBridge_io_pipelinedMemoryBus_rsp_valid),
    .io_pipelinedMemoryBus_rsp_2_data(system_apbBridge_io_pipelinedMemoryBus_rsp_2_data),
    .io_apb_PADDR(system_apbBridge_io_apb_PADDR),
    .io_apb_PSEL(system_apbBridge_io_apb_PSEL),
    .io_apb_PENABLE(system_apbBridge_io_apb_PENABLE),
    .io_apb_PREADY(io_apb_decoder_io_input_PREADY),
    .io_apb_PWRITE(system_apbBridge_io_apb_PWRITE),
    .io_apb_PWDATA(system_apbBridge_io_apb_PWDATA),
    .io_apb_PRDATA(io_apb_decoder_io_input_PRDATA),
    .io_apb_PSLVERROR(io_apb_decoder_io_input_PSLVERROR),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3Gpio system_gpioACtrl ( 
    .io_apb_PADDR(_zz_16_),
    .io_apb_PSEL(apb3Router_1__io_outputs_0_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_0_PENABLE),
    .io_apb_PREADY(system_gpioACtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_0_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_0_PWDATA),
    .io_apb_PRDATA(system_gpioACtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_gpioACtrl_io_apb_PSLVERROR),
    .io_gpio_read(io_gpioA_read),
    .io_gpio_write(system_gpioACtrl_io_gpio_write),
    .io_gpio_writeEnable(system_gpioACtrl_io_gpio_writeEnable),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3UartCtrl system_uartCtrl ( 
    .io_apb_PADDR(_zz_17_),
    .io_apb_PSEL(apb3Router_1__io_outputs_1_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_1_PENABLE),
    .io_apb_PREADY(system_uartCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_1_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_1_PWDATA),
    .io_apb_PRDATA(system_uartCtrl_io_apb_PRDATA),
    .io_uart_txd(system_uartCtrl_io_uart_txd),
    .io_uart_rxd(io_uart_rxd),
    .io_interrupt(system_uartCtrl_io_interrupt),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3PinInterruptCtrl system_pinInterruptCtrl ( 
    .io_apb_PADDR(_zz_18_),
    .io_apb_PSEL(apb3Router_1__io_outputs_2_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_2_PENABLE),
    .io_apb_PREADY(system_pinInterruptCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_2_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_2_PWDATA),
    .io_apb_PRDATA(system_pinInterruptCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_pinInterruptCtrl_io_apb_PSLVERROR),
    .io_pinInterrupt_pins(io_pinInterrupt_pins),
    .io_interrupt(system_pinInterruptCtrl_io_interrupt),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  MuraxApb3Timer system_timer ( 
    .io_apb_PADDR(_zz_19_),
    .io_apb_PSEL(apb3Router_1__io_outputs_3_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_3_PENABLE),
    .io_apb_PREADY(system_timer_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_3_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_3_PWDATA),
    .io_apb_PRDATA(system_timer_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_timer_io_apb_PSLVERROR),
    .io_interrupt(system_timer_io_interrupt),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3PwmCtrl system_pwmCtrl ( 
    .io_apb_PADDR(_zz_20_),
    .io_apb_PSEL(apb3Router_1__io_outputs_4_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_4_PENABLE),
    .io_apb_PREADY(system_pwmCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_4_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_4_PWDATA),
    .io_apb_PRDATA(system_pwmCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_pwmCtrl_io_apb_PSLVERROR),
    .io_pwm_pins(system_pwmCtrl_io_pwm_pins),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3ServoCtrl system_servoCtrl ( 
    .io_apb_PADDR(_zz_21_),
    .io_apb_PSEL(apb3Router_1__io_outputs_5_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_5_PENABLE),
    .io_apb_PREADY(system_servoCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_5_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_5_PWDATA),
    .io_apb_PRDATA(system_servoCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_servoCtrl_io_apb_PSLVERROR),
    .io_servo_pin(system_servoCtrl_io_servo_pin),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3MuxCtrl system_muxCtrl ( 
    .io_apb_PADDR(_zz_22_),
    .io_apb_PSEL(apb3Router_1__io_outputs_6_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_6_PENABLE),
    .io_apb_PREADY(system_muxCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_6_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_6_PWDATA),
    .io_apb_PRDATA(system_muxCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_muxCtrl_io_apb_PSLVERROR),
    .io_mux_pins(system_muxCtrl_io_mux_pins),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3MachineTimerCtrl system_machineTimerCtrl ( 
    .io_apb_PADDR(_zz_23_),
    .io_apb_PSEL(apb3Router_1__io_outputs_7_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_7_PENABLE),
    .io_apb_PREADY(system_machineTimerCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_7_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_7_PWDATA),
    .io_apb_PRDATA(system_machineTimerCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_machineTimerCtrl_io_apb_PSLVERROR),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3ToneCtrl system_toneCtrl ( 
    .io_apb_PADDR(_zz_24_),
    .io_apb_PSEL(apb3Router_1__io_outputs_8_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_8_PENABLE),
    .io_apb_PREADY(system_toneCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_8_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_8_PWDATA),
    .io_apb_PRDATA(system_toneCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_toneCtrl_io_apb_PSLVERROR),
    .io_tone_pin(system_toneCtrl_io_tone_pin),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3ShiftOutCtrl system_shiftOutCtrl ( 
    .io_apb_PADDR(_zz_25_),
    .io_apb_PSEL(apb3Router_1__io_outputs_9_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_9_PENABLE),
    .io_apb_PREADY(system_shiftOutCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_9_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_9_PWDATA),
    .io_apb_PRDATA(system_shiftOutCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_shiftOutCtrl_io_apb_PSLVERROR),
    .io_shiftOut_dataPin(system_shiftOutCtrl_io_shiftOut_dataPin),
    .io_shiftOut_clockPin(system_shiftOutCtrl_io_shiftOut_clockPin),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3SpiMasterCtrl system_spiMasterCtrl ( 
    .io_apb_PADDR(_zz_26_),
    .io_apb_PSEL(apb3Router_1__io_outputs_10_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_10_PENABLE),
    .io_apb_PREADY(system_spiMasterCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_10_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_10_PWDATA),
    .io_apb_PRDATA(system_spiMasterCtrl_io_apb_PRDATA),
    .io_spi_ss(system_spiMasterCtrl_io_spi_ss),
    .io_spi_sclk(system_spiMasterCtrl_io_spi_sclk),
    .io_spi_mosi(system_spiMasterCtrl_io_spi_mosi),
    .io_spi_miso(io_spiMaster_miso),
    .io_interrupt(system_spiMasterCtrl_io_interrupt),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3I2cCtrl system_i2cCtrl ( 
    .io_apb_PADDR(_zz_27_),
    .io_apb_PSEL(apb3Router_1__io_outputs_11_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_11_PENABLE),
    .io_apb_PREADY(system_i2cCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_11_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_11_PWDATA),
    .io_apb_PRDATA(system_i2cCtrl_io_apb_PRDATA),
    .io_i2c_sda_write(system_i2cCtrl_io_i2c_sda_write),
    .io_i2c_sda_read(io_i2c_sda_read),
    .io_i2c_scl_write(system_i2cCtrl_io_i2c_scl_write),
    .io_i2c_scl_read(io_i2c_scl_read),
    .io_interrupt(system_i2cCtrl_io_interrupt),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3PulseInCtrl system_pulseInCtrl ( 
    .io_apb_PADDR(_zz_28_),
    .io_apb_PSEL(apb3Router_1__io_outputs_12_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_12_PENABLE),
    .io_apb_PREADY(system_pulseInCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_12_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_12_PWDATA),
    .io_apb_PRDATA(system_pulseInCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_pulseInCtrl_io_apb_PSLVERROR),
    .io_pulseIn_pin(io_pulseIn_pin),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3SevenSegmentCtrl system_sevenSegmentCtrl ( 
    .io_apb_PADDR(_zz_29_),
    .io_apb_PSEL(apb3Router_1__io_outputs_13_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_13_PENABLE),
    .io_apb_PREADY(system_sevenSegmentCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_13_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_13_PWDATA),
    .io_apb_PRDATA(system_sevenSegmentCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_sevenSegmentCtrl_io_apb_PSLVERROR),
    .io_sevenSegment_digitPin(system_sevenSegmentCtrl_io_sevenSegment_digitPin),
    .io_sevenSegment_segPins(system_sevenSegmentCtrl_io_sevenSegment_segPins),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3ShiftInCtrl system_shiftInCtrl ( 
    .io_apb_PADDR(_zz_30_),
    .io_apb_PSEL(apb3Router_1__io_outputs_14_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_14_PENABLE),
    .io_apb_PREADY(system_shiftInCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_14_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_14_PWDATA),
    .io_apb_PRDATA(system_shiftInCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_shiftInCtrl_io_apb_PSLVERROR),
    .io_shiftIn_dataPin(io_shiftIn_dataPin),
    .io_shiftIn_clockPin(system_shiftInCtrl_io_shiftIn_clockPin),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3QspiCtrl system_qspiCtrl ( 
    .io_apb_PADDR(_zz_31_),
    .io_apb_PSEL(apb3Router_1__io_outputs_15_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_15_PENABLE),
    .io_apb_PREADY(system_qspiCtrl_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_15_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_15_PWDATA),
    .io_apb_PRDATA(system_qspiCtrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(system_qspiCtrl_io_apb_PSLVERROR),
    .io_qspi_qck(io_qspi_qck),
    .io_qspi_qss(io_qspi_qss),
    .io_qspi_qd_read(io_qspi_qd_read),
    .io_qspi_qd_write(system_qspiCtrl_io_qspi_qd_write),
    .io_qspi_qd_writeEnable(system_qspiCtrl_io_qspi_qd_writeEnable),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Apb3Decoder io_apb_decoder ( 
    .io_input_PADDR(system_apbBridge_io_apb_PADDR),
    .io_input_PSEL(system_apbBridge_io_apb_PSEL),
    .io_input_PENABLE(system_apbBridge_io_apb_PENABLE),
    .io_input_PREADY(io_apb_decoder_io_input_PREADY),
    .io_input_PWRITE(system_apbBridge_io_apb_PWRITE),
    .io_input_PWDATA(system_apbBridge_io_apb_PWDATA),
    .io_input_PRDATA(io_apb_decoder_io_input_PRDATA),
    .io_input_PSLVERROR(io_apb_decoder_io_input_PSLVERROR),
    .io_output_PADDR(io_apb_decoder_io_output_PADDR),
    .io_output_PSEL(io_apb_decoder_io_output_PSEL),
    .io_output_PENABLE(io_apb_decoder_io_output_PENABLE),
    .io_output_PREADY(apb3Router_1__io_input_PREADY),
    .io_output_PWRITE(io_apb_decoder_io_output_PWRITE),
    .io_output_PWDATA(io_apb_decoder_io_output_PWDATA),
    .io_output_PRDATA(apb3Router_1__io_input_PRDATA),
    .io_output_PSLVERROR(apb3Router_1__io_input_PSLVERROR) 
  );
  Apb3Router apb3Router_1_ ( 
    .io_input_PADDR(io_apb_decoder_io_output_PADDR),
    .io_input_PSEL(io_apb_decoder_io_output_PSEL),
    .io_input_PENABLE(io_apb_decoder_io_output_PENABLE),
    .io_input_PREADY(apb3Router_1__io_input_PREADY),
    .io_input_PWRITE(io_apb_decoder_io_output_PWRITE),
    .io_input_PWDATA(io_apb_decoder_io_output_PWDATA),
    .io_input_PRDATA(apb3Router_1__io_input_PRDATA),
    .io_input_PSLVERROR(apb3Router_1__io_input_PSLVERROR),
    .io_outputs_0_PADDR(apb3Router_1__io_outputs_0_PADDR),
    .io_outputs_0_PSEL(apb3Router_1__io_outputs_0_PSEL),
    .io_outputs_0_PENABLE(apb3Router_1__io_outputs_0_PENABLE),
    .io_outputs_0_PREADY(system_gpioACtrl_io_apb_PREADY),
    .io_outputs_0_PWRITE(apb3Router_1__io_outputs_0_PWRITE),
    .io_outputs_0_PWDATA(apb3Router_1__io_outputs_0_PWDATA),
    .io_outputs_0_PRDATA(system_gpioACtrl_io_apb_PRDATA),
    .io_outputs_0_PSLVERROR(system_gpioACtrl_io_apb_PSLVERROR),
    .io_outputs_1_PADDR(apb3Router_1__io_outputs_1_PADDR),
    .io_outputs_1_PSEL(apb3Router_1__io_outputs_1_PSEL),
    .io_outputs_1_PENABLE(apb3Router_1__io_outputs_1_PENABLE),
    .io_outputs_1_PREADY(system_uartCtrl_io_apb_PREADY),
    .io_outputs_1_PWRITE(apb3Router_1__io_outputs_1_PWRITE),
    .io_outputs_1_PWDATA(apb3Router_1__io_outputs_1_PWDATA),
    .io_outputs_1_PRDATA(system_uartCtrl_io_apb_PRDATA),
    .io_outputs_1_PSLVERROR(_zz_32_),
    .io_outputs_2_PADDR(apb3Router_1__io_outputs_2_PADDR),
    .io_outputs_2_PSEL(apb3Router_1__io_outputs_2_PSEL),
    .io_outputs_2_PENABLE(apb3Router_1__io_outputs_2_PENABLE),
    .io_outputs_2_PREADY(system_pinInterruptCtrl_io_apb_PREADY),
    .io_outputs_2_PWRITE(apb3Router_1__io_outputs_2_PWRITE),
    .io_outputs_2_PWDATA(apb3Router_1__io_outputs_2_PWDATA),
    .io_outputs_2_PRDATA(system_pinInterruptCtrl_io_apb_PRDATA),
    .io_outputs_2_PSLVERROR(system_pinInterruptCtrl_io_apb_PSLVERROR),
    .io_outputs_3_PADDR(apb3Router_1__io_outputs_3_PADDR),
    .io_outputs_3_PSEL(apb3Router_1__io_outputs_3_PSEL),
    .io_outputs_3_PENABLE(apb3Router_1__io_outputs_3_PENABLE),
    .io_outputs_3_PREADY(system_timer_io_apb_PREADY),
    .io_outputs_3_PWRITE(apb3Router_1__io_outputs_3_PWRITE),
    .io_outputs_3_PWDATA(apb3Router_1__io_outputs_3_PWDATA),
    .io_outputs_3_PRDATA(system_timer_io_apb_PRDATA),
    .io_outputs_3_PSLVERROR(system_timer_io_apb_PSLVERROR),
    .io_outputs_4_PADDR(apb3Router_1__io_outputs_4_PADDR),
    .io_outputs_4_PSEL(apb3Router_1__io_outputs_4_PSEL),
    .io_outputs_4_PENABLE(apb3Router_1__io_outputs_4_PENABLE),
    .io_outputs_4_PREADY(system_pwmCtrl_io_apb_PREADY),
    .io_outputs_4_PWRITE(apb3Router_1__io_outputs_4_PWRITE),
    .io_outputs_4_PWDATA(apb3Router_1__io_outputs_4_PWDATA),
    .io_outputs_4_PRDATA(system_pwmCtrl_io_apb_PRDATA),
    .io_outputs_4_PSLVERROR(system_pwmCtrl_io_apb_PSLVERROR),
    .io_outputs_5_PADDR(apb3Router_1__io_outputs_5_PADDR),
    .io_outputs_5_PSEL(apb3Router_1__io_outputs_5_PSEL),
    .io_outputs_5_PENABLE(apb3Router_1__io_outputs_5_PENABLE),
    .io_outputs_5_PREADY(system_servoCtrl_io_apb_PREADY),
    .io_outputs_5_PWRITE(apb3Router_1__io_outputs_5_PWRITE),
    .io_outputs_5_PWDATA(apb3Router_1__io_outputs_5_PWDATA),
    .io_outputs_5_PRDATA(system_servoCtrl_io_apb_PRDATA),
    .io_outputs_5_PSLVERROR(system_servoCtrl_io_apb_PSLVERROR),
    .io_outputs_6_PADDR(apb3Router_1__io_outputs_6_PADDR),
    .io_outputs_6_PSEL(apb3Router_1__io_outputs_6_PSEL),
    .io_outputs_6_PENABLE(apb3Router_1__io_outputs_6_PENABLE),
    .io_outputs_6_PREADY(system_muxCtrl_io_apb_PREADY),
    .io_outputs_6_PWRITE(apb3Router_1__io_outputs_6_PWRITE),
    .io_outputs_6_PWDATA(apb3Router_1__io_outputs_6_PWDATA),
    .io_outputs_6_PRDATA(system_muxCtrl_io_apb_PRDATA),
    .io_outputs_6_PSLVERROR(system_muxCtrl_io_apb_PSLVERROR),
    .io_outputs_7_PADDR(apb3Router_1__io_outputs_7_PADDR),
    .io_outputs_7_PSEL(apb3Router_1__io_outputs_7_PSEL),
    .io_outputs_7_PENABLE(apb3Router_1__io_outputs_7_PENABLE),
    .io_outputs_7_PREADY(system_machineTimerCtrl_io_apb_PREADY),
    .io_outputs_7_PWRITE(apb3Router_1__io_outputs_7_PWRITE),
    .io_outputs_7_PWDATA(apb3Router_1__io_outputs_7_PWDATA),
    .io_outputs_7_PRDATA(system_machineTimerCtrl_io_apb_PRDATA),
    .io_outputs_7_PSLVERROR(system_machineTimerCtrl_io_apb_PSLVERROR),
    .io_outputs_8_PADDR(apb3Router_1__io_outputs_8_PADDR),
    .io_outputs_8_PSEL(apb3Router_1__io_outputs_8_PSEL),
    .io_outputs_8_PENABLE(apb3Router_1__io_outputs_8_PENABLE),
    .io_outputs_8_PREADY(system_toneCtrl_io_apb_PREADY),
    .io_outputs_8_PWRITE(apb3Router_1__io_outputs_8_PWRITE),
    .io_outputs_8_PWDATA(apb3Router_1__io_outputs_8_PWDATA),
    .io_outputs_8_PRDATA(system_toneCtrl_io_apb_PRDATA),
    .io_outputs_8_PSLVERROR(system_toneCtrl_io_apb_PSLVERROR),
    .io_outputs_9_PADDR(apb3Router_1__io_outputs_9_PADDR),
    .io_outputs_9_PSEL(apb3Router_1__io_outputs_9_PSEL),
    .io_outputs_9_PENABLE(apb3Router_1__io_outputs_9_PENABLE),
    .io_outputs_9_PREADY(system_shiftOutCtrl_io_apb_PREADY),
    .io_outputs_9_PWRITE(apb3Router_1__io_outputs_9_PWRITE),
    .io_outputs_9_PWDATA(apb3Router_1__io_outputs_9_PWDATA),
    .io_outputs_9_PRDATA(system_shiftOutCtrl_io_apb_PRDATA),
    .io_outputs_9_PSLVERROR(system_shiftOutCtrl_io_apb_PSLVERROR),
    .io_outputs_10_PADDR(apb3Router_1__io_outputs_10_PADDR),
    .io_outputs_10_PSEL(apb3Router_1__io_outputs_10_PSEL),
    .io_outputs_10_PENABLE(apb3Router_1__io_outputs_10_PENABLE),
    .io_outputs_10_PREADY(system_spiMasterCtrl_io_apb_PREADY),
    .io_outputs_10_PWRITE(apb3Router_1__io_outputs_10_PWRITE),
    .io_outputs_10_PWDATA(apb3Router_1__io_outputs_10_PWDATA),
    .io_outputs_10_PRDATA(system_spiMasterCtrl_io_apb_PRDATA),
    .io_outputs_10_PSLVERROR(_zz_33_),
    .io_outputs_11_PADDR(apb3Router_1__io_outputs_11_PADDR),
    .io_outputs_11_PSEL(apb3Router_1__io_outputs_11_PSEL),
    .io_outputs_11_PENABLE(apb3Router_1__io_outputs_11_PENABLE),
    .io_outputs_11_PREADY(system_i2cCtrl_io_apb_PREADY),
    .io_outputs_11_PWRITE(apb3Router_1__io_outputs_11_PWRITE),
    .io_outputs_11_PWDATA(apb3Router_1__io_outputs_11_PWDATA),
    .io_outputs_11_PRDATA(system_i2cCtrl_io_apb_PRDATA),
    .io_outputs_11_PSLVERROR(_zz_34_),
    .io_outputs_12_PADDR(apb3Router_1__io_outputs_12_PADDR),
    .io_outputs_12_PSEL(apb3Router_1__io_outputs_12_PSEL),
    .io_outputs_12_PENABLE(apb3Router_1__io_outputs_12_PENABLE),
    .io_outputs_12_PREADY(system_pulseInCtrl_io_apb_PREADY),
    .io_outputs_12_PWRITE(apb3Router_1__io_outputs_12_PWRITE),
    .io_outputs_12_PWDATA(apb3Router_1__io_outputs_12_PWDATA),
    .io_outputs_12_PRDATA(system_pulseInCtrl_io_apb_PRDATA),
    .io_outputs_12_PSLVERROR(system_pulseInCtrl_io_apb_PSLVERROR),
    .io_outputs_13_PADDR(apb3Router_1__io_outputs_13_PADDR),
    .io_outputs_13_PSEL(apb3Router_1__io_outputs_13_PSEL),
    .io_outputs_13_PENABLE(apb3Router_1__io_outputs_13_PENABLE),
    .io_outputs_13_PREADY(system_sevenSegmentCtrl_io_apb_PREADY),
    .io_outputs_13_PWRITE(apb3Router_1__io_outputs_13_PWRITE),
    .io_outputs_13_PWDATA(apb3Router_1__io_outputs_13_PWDATA),
    .io_outputs_13_PRDATA(system_sevenSegmentCtrl_io_apb_PRDATA),
    .io_outputs_13_PSLVERROR(system_sevenSegmentCtrl_io_apb_PSLVERROR),
    .io_outputs_14_PADDR(apb3Router_1__io_outputs_14_PADDR),
    .io_outputs_14_PSEL(apb3Router_1__io_outputs_14_PSEL),
    .io_outputs_14_PENABLE(apb3Router_1__io_outputs_14_PENABLE),
    .io_outputs_14_PREADY(system_shiftInCtrl_io_apb_PREADY),
    .io_outputs_14_PWRITE(apb3Router_1__io_outputs_14_PWRITE),
    .io_outputs_14_PWDATA(apb3Router_1__io_outputs_14_PWDATA),
    .io_outputs_14_PRDATA(system_shiftInCtrl_io_apb_PRDATA),
    .io_outputs_14_PSLVERROR(system_shiftInCtrl_io_apb_PSLVERROR),
    .io_outputs_15_PADDR(apb3Router_1__io_outputs_15_PADDR),
    .io_outputs_15_PSEL(apb3Router_1__io_outputs_15_PSEL),
    .io_outputs_15_PENABLE(apb3Router_1__io_outputs_15_PENABLE),
    .io_outputs_15_PREADY(system_qspiCtrl_io_apb_PREADY),
    .io_outputs_15_PWRITE(apb3Router_1__io_outputs_15_PWRITE),
    .io_outputs_15_PWDATA(apb3Router_1__io_outputs_15_PWDATA),
    .io_outputs_15_PRDATA(system_qspiCtrl_io_apb_PRDATA),
    .io_outputs_15_PSLVERROR(system_qspiCtrl_io_apb_PSLVERROR),
    .toplevel_io_mainClk(io_mainClk),
    .toplevel_resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  always @(*) begin
    case(system_mainBusDecoder_logic_rspSourceId)
      2'b00 : begin
        _zz_35_ = system_ram_io_bus_rsp_0_data;
      end
      2'b01 : begin
        _zz_35_ = system_sramCtrl_io_bus_rsp_1_data;
      end
      default : begin
        _zz_35_ = system_apbBridge_io_pipelinedMemoryBus_rsp_2_data;
      end
    endcase
  end

  always @ (*) begin
    resetCtrl_mainClkResetUnbuffered = 1'b0;
    if(_zz_36_)begin
      resetCtrl_mainClkResetUnbuffered = 1'b1;
    end
  end

  assign _zz_1_[5 : 0] = (6'b111111);
  always @ (*) begin
    system_timerInterrupt = 1'b0;
    if(system_timer_io_interrupt)begin
      system_timerInterrupt = 1'b1;
    end
  end

  always @ (*) begin
    system_externalInterrupt = 1'b0;
    if(system_uartCtrl_io_interrupt)begin
      system_externalInterrupt = 1'b1;
    end
    if(system_pinInterruptCtrl_io_interrupt)begin
      system_externalInterrupt = 1'b1;
    end
  end

  assign system_cpu_dBus_cmd_halfPipe_valid = system_cpu_dBus_cmd_halfPipe_regs_valid;
  assign system_cpu_dBus_cmd_halfPipe_payload_wr = system_cpu_dBus_cmd_halfPipe_regs_payload_wr;
  assign system_cpu_dBus_cmd_halfPipe_payload_address = system_cpu_dBus_cmd_halfPipe_regs_payload_address;
  assign system_cpu_dBus_cmd_halfPipe_payload_data = system_cpu_dBus_cmd_halfPipe_regs_payload_data;
  assign system_cpu_dBus_cmd_halfPipe_payload_size = system_cpu_dBus_cmd_halfPipe_regs_payload_size;
  assign system_cpu_dBus_cmd_halfPipe_ready = system_mainBusArbiter_io_dBus_cmd_ready;
  assign _zz_12_ = systemDebugger_1__io_mem_cmd_payload_address[7:0];
  assign io_jtag_tdo = jtagBridge_1__io_jtag_tdo;
  assign io_sram_addr = system_sramCtrl_io_sram_addr;
  assign io_sram_dat_write = system_sramCtrl_io_sram_dat_write;
  assign io_sram_dat_writeEnable = system_sramCtrl_io_sram_dat_writeEnable;
  assign io_sram_cs = system_sramCtrl_io_sram_cs;
  assign io_sram_we = system_sramCtrl_io_sram_we;
  assign io_sram_oe = system_sramCtrl_io_sram_oe;
  assign io_sram_lb = system_sramCtrl_io_sram_lb;
  assign io_sram_ub = system_sramCtrl_io_sram_ub;
  assign io_gpioA_write = system_gpioACtrl_io_gpio_write;
  assign io_gpioA_writeEnable = system_gpioACtrl_io_gpio_writeEnable;
  assign io_uart_txd = system_uartCtrl_io_uart_txd;
  assign io_pwm_pins = system_pwmCtrl_io_pwm_pins;
  assign io_servo_pin = system_servoCtrl_io_servo_pin;
  assign io_mux_pins = system_muxCtrl_io_mux_pins;
  assign io_tone_pin = system_toneCtrl_io_tone_pin;
  assign io_shiftOut_dataPin = system_shiftOutCtrl_io_shiftOut_dataPin;
  assign io_shiftOut_clockPin = system_shiftOutCtrl_io_shiftOut_clockPin;
  assign io_spiMaster_ss = system_spiMasterCtrl_io_spi_ss;
  assign io_spiMaster_sclk = system_spiMasterCtrl_io_spi_sclk;
  assign io_spiMaster_mosi = system_spiMasterCtrl_io_spi_mosi;
  assign io_i2c_sda_write = system_i2cCtrl_io_i2c_sda_write;
  assign io_i2c_scl_write = system_i2cCtrl_io_i2c_scl_write;
  assign io_sevenSegment_digitPin = system_sevenSegmentCtrl_io_sevenSegment_digitPin;
  assign io_sevenSegment_segPins = system_sevenSegmentCtrl_io_sevenSegment_segPins;
  assign io_shiftIn_clockPin = system_shiftInCtrl_io_shiftIn_clockPin;
  assign io_qspi_qd_write = system_qspiCtrl_io_qspi_qd_write;
  assign io_qspi_qd_writeEnable = system_qspiCtrl_io_qspi_qd_writeEnable;
  assign _zz_16_ = apb3Router_1__io_outputs_0_PADDR[3:0];
  assign _zz_17_ = apb3Router_1__io_outputs_1_PADDR[3:0];
  assign _zz_32_ = 1'b0;
  assign _zz_18_ = apb3Router_1__io_outputs_2_PADDR[7:0];
  assign _zz_19_ = apb3Router_1__io_outputs_3_PADDR[7:0];
  assign _zz_20_ = apb3Router_1__io_outputs_4_PADDR[7:0];
  assign _zz_21_ = apb3Router_1__io_outputs_5_PADDR[7:0];
  assign _zz_22_ = apb3Router_1__io_outputs_6_PADDR[7:0];
  assign _zz_23_ = apb3Router_1__io_outputs_7_PADDR[7:0];
  assign _zz_24_ = apb3Router_1__io_outputs_8_PADDR[7:0];
  assign _zz_25_ = apb3Router_1__io_outputs_9_PADDR[7:0];
  assign _zz_26_ = apb3Router_1__io_outputs_10_PADDR[7:0];
  assign _zz_33_ = 1'b0;
  assign _zz_27_ = apb3Router_1__io_outputs_11_PADDR[7:0];
  assign _zz_34_ = 1'b0;
  assign _zz_28_ = apb3Router_1__io_outputs_12_PADDR[7:0];
  assign _zz_29_ = apb3Router_1__io_outputs_13_PADDR[7:0];
  assign _zz_30_ = apb3Router_1__io_outputs_14_PADDR[7:0];
  assign _zz_31_ = apb3Router_1__io_outputs_15_PADDR[7:0];
  assign system_mainBusArbiter_io_masterBus_cmd_s2mPipe_valid = (system_mainBusArbiter_io_masterBus_cmd_valid || _zz_3_);
  assign _zz_11_ = (! _zz_3_);
  assign system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_write = (_zz_3_ ? _zz_4_ : system_mainBusArbiter_io_masterBus_cmd_payload_write);
  assign system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_address = (_zz_3_ ? _zz_5_ : system_mainBusArbiter_io_masterBus_cmd_payload_address);
  assign system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_data = (_zz_3_ ? _zz_6_ : system_mainBusArbiter_io_masterBus_cmd_payload_data);
  assign system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_mask = (_zz_3_ ? _zz_7_ : system_mainBusArbiter_io_masterBus_cmd_payload_mask);
  assign system_mainBusArbiter_io_masterBus_cmd_s2mPipe_ready = system_mainBusDecoder_logic_masterPipelined_cmd_ready;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_valid = system_mainBusArbiter_io_masterBus_cmd_s2mPipe_valid;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_write = system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_write;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_address = system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_address;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_data = system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_data;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask = system_mainBusArbiter_io_masterBus_cmd_s2mPipe_payload_mask;
  assign system_mainBusDecoder_logic_hits_0 = (((32'b10000000000000000000000000000000) <= system_mainBusDecoder_logic_masterPipelined_cmd_payload_address) && (system_mainBusDecoder_logic_masterPipelined_cmd_payload_address < (32'b10000000000000000011000000000000)));
  always @ (*) begin
    _zz_13_ = (system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_hits_0);
    _zz_14_ = (system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_hits_1);
    _zz_15_ = (system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_hits_2);
    system_mainBusDecoder_logic_masterPipelined_cmd_ready = (({(system_mainBusDecoder_logic_hits_2 && system_apbBridge_io_pipelinedMemoryBus_cmd_ready),{(system_mainBusDecoder_logic_hits_1 && system_sramCtrl_io_bus_cmd_ready),(system_mainBusDecoder_logic_hits_0 && system_ram_io_bus_cmd_ready)}} != (3'b000)) || system_mainBusDecoder_logic_noHit);
    if((system_mainBusDecoder_logic_rspPending && (! system_mainBusDecoder_logic_masterPipelined_rsp_valid)))begin
      system_mainBusDecoder_logic_masterPipelined_cmd_ready = 1'b0;
      _zz_13_ = 1'b0;
      _zz_14_ = 1'b0;
      _zz_15_ = 1'b0;
    end
  end

  assign _zz_8_ = system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  assign system_mainBusDecoder_logic_hits_1 = ((system_mainBusDecoder_logic_masterPipelined_cmd_payload_address & _zz_39_) == (32'b10010000000000000000000000000000));
  assign _zz_9_ = system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  assign system_mainBusDecoder_logic_hits_2 = ((system_mainBusDecoder_logic_masterPipelined_cmd_payload_address & _zz_40_) == (32'b11110000000000000000000000000000));
  assign _zz_10_ = system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  assign system_mainBusDecoder_logic_noHit = (! ({system_mainBusDecoder_logic_hits_2,{system_mainBusDecoder_logic_hits_1,system_mainBusDecoder_logic_hits_0}} != (3'b000)));
  assign system_mainBusDecoder_logic_masterPipelined_rsp_valid = (({system_apbBridge_io_pipelinedMemoryBus_rsp_valid,{system_sramCtrl_io_bus_rsp_valid,system_ram_io_bus_rsp_valid}} != (3'b000)) || (system_mainBusDecoder_logic_rspPending && system_mainBusDecoder_logic_rspNoHit));
  assign system_mainBusDecoder_logic_masterPipelined_rsp_payload_data = _zz_35_;
  always @ (posedge io_mainClk) begin
    if(_zz_36_)begin
      resetCtrl_systemClkResetCounter <= (resetCtrl_systemClkResetCounter + (6'b000001));
    end
    if(bufferCC_5__io_dataOut)begin
      resetCtrl_systemClkResetCounter <= (6'b000000);
    end
  end

  always @ (posedge io_mainClk) begin
    resetCtrl_mainClkReset <= resetCtrl_mainClkResetUnbuffered;
    resetCtrl_systemReset <= resetCtrl_mainClkResetUnbuffered;
    if(system_cpu_debug_resetOut_regNext)begin
      resetCtrl_systemReset <= 1'b1;
    end
  end

  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      system_cpu_dBus_cmd_halfPipe_regs_valid <= 1'b0;
      system_cpu_dBus_cmd_halfPipe_regs_ready <= 1'b1;
      _zz_3_ <= 1'b0;
      system_mainBusDecoder_logic_rspPending <= 1'b0;
      system_mainBusDecoder_logic_rspNoHit <= 1'b0;
    end else begin
      if(_zz_37_)begin
        system_cpu_dBus_cmd_halfPipe_regs_valid <= system_cpu_dBus_cmd_valid;
        system_cpu_dBus_cmd_halfPipe_regs_ready <= (! system_cpu_dBus_cmd_valid);
      end else begin
        system_cpu_dBus_cmd_halfPipe_regs_valid <= (! system_cpu_dBus_cmd_halfPipe_ready);
        system_cpu_dBus_cmd_halfPipe_regs_ready <= system_cpu_dBus_cmd_halfPipe_ready;
      end
      if(system_mainBusArbiter_io_masterBus_cmd_s2mPipe_ready)begin
        _zz_3_ <= 1'b0;
      end
      if(_zz_38_)begin
        _zz_3_ <= system_mainBusArbiter_io_masterBus_cmd_valid;
      end
      if(system_mainBusDecoder_logic_masterPipelined_rsp_valid)begin
        system_mainBusDecoder_logic_rspPending <= 1'b0;
      end
      if(((system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_masterPipelined_cmd_ready) && (! system_mainBusDecoder_logic_masterPipelined_cmd_payload_write)))begin
        system_mainBusDecoder_logic_rspPending <= 1'b1;
      end
      system_mainBusDecoder_logic_rspNoHit <= 1'b0;
      if(system_mainBusDecoder_logic_noHit)begin
        system_mainBusDecoder_logic_rspNoHit <= 1'b1;
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_37_)begin
      system_cpu_dBus_cmd_halfPipe_regs_payload_wr <= system_cpu_dBus_cmd_payload_wr;
      system_cpu_dBus_cmd_halfPipe_regs_payload_address <= system_cpu_dBus_cmd_payload_address;
      system_cpu_dBus_cmd_halfPipe_regs_payload_data <= system_cpu_dBus_cmd_payload_data;
      system_cpu_dBus_cmd_halfPipe_regs_payload_size <= system_cpu_dBus_cmd_payload_size;
    end
    if(_zz_38_)begin
      _zz_4_ <= system_mainBusArbiter_io_masterBus_cmd_payload_write;
      _zz_5_ <= system_mainBusArbiter_io_masterBus_cmd_payload_address;
      _zz_6_ <= system_mainBusArbiter_io_masterBus_cmd_payload_data;
      _zz_7_ <= system_mainBusArbiter_io_masterBus_cmd_payload_mask;
    end
    if((system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_masterPipelined_cmd_ready))begin
      system_mainBusDecoder_logic_rspSourceId <= {system_mainBusDecoder_logic_hits_2,system_mainBusDecoder_logic_hits_1};
    end
  end

  always @ (posedge io_mainClk) begin
    system_cpu_debug_resetOut_regNext <= system_cpu_debug_resetOut;
  end

  always @ (posedge io_mainClk or posedge resetCtrl_mainClkReset) begin
    if (resetCtrl_mainClkReset) begin
      _zz_2_ <= 1'b0;
    end else begin
      _zz_2_ <= (systemDebugger_1__io_mem_cmd_valid && system_cpu_debug_bus_cmd_ready);
    end
  end

endmodule

