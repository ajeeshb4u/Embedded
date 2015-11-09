
_read_ds1307:
;dc.c,19 :: 		unsigned short read_ds1307(unsigned short address)
;dc.c,22 :: 		I2C1_Start();
	CALL       _I2C1_Start+0
;dc.c,23 :: 		I2C1_Wr(0xD0); //address 0x68 followed by direction bit (0 for write, 1 for read) 0x68 followed by 0 --> 0xD0
	MOVLW      208
	MOVWF      FARG_I2C1_Wr_data_+0
	CALL       _I2C1_Wr+0
;dc.c,24 :: 		I2C1_Wr(address);
	MOVF       FARG_read_ds1307_address+0, 0
	MOVWF      FARG_I2C1_Wr_data_+0
	CALL       _I2C1_Wr+0
;dc.c,25 :: 		I2C1_Repeated_Start();
	CALL       _I2C1_Repeated_Start+0
;dc.c,26 :: 		I2C1_Wr(0xD1); //0x68 followed by 1 --> 0xD1
	MOVLW      209
	MOVWF      FARG_I2C1_Wr_data_+0
	CALL       _I2C1_Wr+0
;dc.c,27 :: 		r_data=I2C1_Rd(0);
	CLRF       FARG_I2C1_Rd_ack+0
	CALL       _I2C1_Rd+0
	MOVF       R0+0, 0
	MOVWF      read_ds1307_r_data_L0+0
;dc.c,28 :: 		I2C1_Stop();
	CALL       _I2C1_Stop+0
;dc.c,29 :: 		return(r_data);
	MOVF       read_ds1307_r_data_L0+0, 0
	MOVWF      R0+0
;dc.c,30 :: 		}
	RETURN
; end of _read_ds1307

_write_ds1307:
;dc.c,33 :: 		void write_ds1307(unsigned short address,unsigned short w_data)
;dc.c,35 :: 		I2C1_Start(); // issue I2C start signal
	CALL       _I2C1_Start+0
;dc.c,37 :: 		I2C1_Wr(0xD0); // send byte via I2C (device address + W)
	MOVLW      208
	MOVWF      FARG_I2C1_Wr_data_+0
	CALL       _I2C1_Wr+0
;dc.c,38 :: 		I2C1_Wr(address); // send byte (address of DS1307 location)
	MOVF       FARG_write_ds1307_address+0, 0
	MOVWF      FARG_I2C1_Wr_data_+0
	CALL       _I2C1_Wr+0
;dc.c,39 :: 		I2C1_Wr(w_data); // send data (data to be written)
	MOVF       FARG_write_ds1307_w_data+0, 0
	MOVWF      FARG_I2C1_Wr_data_+0
	CALL       _I2C1_Wr+0
;dc.c,40 :: 		I2C1_Stop(); // issue I2C stop signal
	CALL       _I2C1_Stop+0
;dc.c,41 :: 		}
	RETURN
; end of _write_ds1307

_BCD2UpperCh:
;dc.c,44 :: 		unsigned char BCD2UpperCh(unsigned char bcd)
;dc.c,46 :: 		return ((bcd >> 4) + '0');
	MOVF       FARG_BCD2UpperCh_bcd+0, 0
	MOVWF      R0+0
	RRF        R0+0, 1
	BCF        R0+0, 7
	RRF        R0+0, 1
	BCF        R0+0, 7
	RRF        R0+0, 1
	BCF        R0+0, 7
	RRF        R0+0, 1
	BCF        R0+0, 7
	MOVLW      48
	ADDWF      R0+0, 1
;dc.c,47 :: 		}
	RETURN
; end of _BCD2UpperCh

_BCD2LowerCh:
;dc.c,50 :: 		unsigned char BCD2LowerCh(unsigned char bcd)
;dc.c,52 :: 		return ((bcd & 0x0F) + '0');
	MOVLW      15
	ANDWF      FARG_BCD2LowerCh_bcd+0, 0
	MOVWF      R0+0
	MOVLW      48
	ADDWF      R0+0, 1
;dc.c,53 :: 		}
	RETURN
; end of _BCD2LowerCh

_Binary2BCD:
;dc.c,56 :: 		int Binary2BCD(int a)
;dc.c,59 :: 		t1 = a%10;
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVF       FARG_Binary2BCD_a+0, 0
	MOVWF      R0+0
	MOVF       FARG_Binary2BCD_a+1, 0
	MOVWF      R0+1
	CALL       _Div_16x16_S+0
	MOVF       R8+0, 0
	MOVWF      R0+0
	MOVF       R8+1, 0
	MOVWF      R0+1
	MOVF       R0+0, 0
	MOVWF      Binary2BCD_t1_L0+0
	MOVF       R0+1, 0
	MOVWF      Binary2BCD_t1_L0+1
;dc.c,60 :: 		t1 = t1 & 0x0F;
	MOVLW      15
	ANDWF      R0+0, 0
	MOVWF      Binary2BCD_t1_L0+0
	MOVF       R0+1, 0
	MOVWF      Binary2BCD_t1_L0+1
	MOVLW      0
	ANDWF      Binary2BCD_t1_L0+1, 1
;dc.c,61 :: 		a = a/10;
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVF       FARG_Binary2BCD_a+0, 0
	MOVWF      R0+0
	MOVF       FARG_Binary2BCD_a+1, 0
	MOVWF      R0+1
	CALL       _Div_16x16_S+0
	MOVF       R0+0, 0
	MOVWF      FARG_Binary2BCD_a+0
	MOVF       R0+1, 0
	MOVWF      FARG_Binary2BCD_a+1
;dc.c,62 :: 		t2 = a%10;
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Div_16x16_S+0
	MOVF       R8+0, 0
	MOVWF      R0+0
	MOVF       R8+1, 0
	MOVWF      R0+1
;dc.c,63 :: 		t2 = 0x0F & t2;
	MOVLW      15
	ANDWF      R0+0, 0
	MOVWF      R3+0
	MOVF       R0+1, 0
	MOVWF      R3+1
	MOVLW      0
	ANDWF      R3+1, 1
;dc.c,64 :: 		t2 = t2 << 4;
	MOVLW      4
	MOVWF      R2+0
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__Binary2BCD40:
	BTFSC      STATUS+0, 2
	GOTO       L__Binary2BCD41
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__Binary2BCD40
L__Binary2BCD41:
;dc.c,65 :: 		t2 = 0xF0 & t2;
	MOVLW      240
	ANDWF      R0+0, 1
	MOVLW      0
	ANDWF      R0+1, 1
;dc.c,66 :: 		t1 = t1 | t2;
	MOVF       Binary2BCD_t1_L0+0, 0
	IORWF      R0+0, 1
	MOVF       Binary2BCD_t1_L0+1, 0
	IORWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      Binary2BCD_t1_L0+0
	MOVF       R0+1, 0
	MOVWF      Binary2BCD_t1_L0+1
;dc.c,67 :: 		return t1;
;dc.c,68 :: 		}
	RETURN
; end of _Binary2BCD

_BCD2Binary:
;dc.c,71 :: 		int BCD2Binary(int a)
;dc.c,74 :: 		t = a & 0x0F;
	MOVLW      15
	ANDWF      FARG_BCD2Binary_a+0, 0
	MOVWF      BCD2Binary_r_L0+0
	MOVF       FARG_BCD2Binary_a+1, 0
	MOVWF      BCD2Binary_r_L0+1
	MOVLW      0
	ANDWF      BCD2Binary_r_L0+1, 1
;dc.c,75 :: 		r = t;
;dc.c,76 :: 		a = 0xF0 & a;
	MOVLW      240
	ANDWF      FARG_BCD2Binary_a+0, 0
	MOVWF      R3+0
	MOVF       FARG_BCD2Binary_a+1, 0
	MOVWF      R3+1
	MOVLW      0
	ANDWF      R3+1, 1
	MOVF       R3+0, 0
	MOVWF      FARG_BCD2Binary_a+0
	MOVF       R3+1, 0
	MOVWF      FARG_BCD2Binary_a+1
;dc.c,77 :: 		t = a >> 4;
	MOVLW      4
	MOVWF      R2+0
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__BCD2Binary42:
	BTFSC      STATUS+0, 2
	GOTO       L__BCD2Binary43
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	ADDLW      255
	GOTO       L__BCD2Binary42
L__BCD2Binary43:
;dc.c,78 :: 		t = 0x0F & t;
	MOVLW      15
	ANDWF      R0+0, 1
	MOVLW      0
	ANDWF      R0+1, 1
;dc.c,79 :: 		r = t*10 + r;
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Mul_16x16_U+0
	MOVF       BCD2Binary_r_L0+0, 0
	ADDWF      R0+0, 1
	MOVF       BCD2Binary_r_L0+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      BCD2Binary_r_L0+0
	MOVF       R0+1, 0
	MOVWF      BCD2Binary_r_L0+1
;dc.c,80 :: 		return r;
;dc.c,81 :: 		}
	RETURN
; end of _BCD2Binary

_main:
;dc.c,101 :: 		void main()
;dc.c,103 :: 		I2C1_Init(100000); //DS1307 I2C is running at 100KHz
	MOVLW      20
	MOVWF      SSPADD+0
	CALL       _I2C1_Init+0
;dc.c,105 :: 		CMCON = 0x07;   // To turn off comparators
	MOVLW      7
	MOVWF      CMCON+0
;dc.c,106 :: 		ADCON1 = 0x06;  // To turn off analog to digital converters
	MOVLW      6
	MOVWF      ADCON1+0
;dc.c,108 :: 		TRISA = 0x07;
	MOVLW      7
	MOVWF      TRISA+0
;dc.c,109 :: 		PORTA = 0x00;
	CLRF       PORTA+0
;dc.c,111 :: 		Lcd_Init();                        // Initialize LCD
	CALL       _Lcd_Init+0
;dc.c,112 :: 		Lcd_Cmd(_LCD_CLEAR);               // Clear display
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;dc.c,113 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);          // Cursor off
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;dc.c,114 :: 		Lcd_out(1,1,"Time:");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr1_dc+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;dc.c,115 :: 		Lcd_out(2,1,"Date:");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr2_dc+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;dc.c,117 :: 		do
L_main0:
;dc.c,119 :: 		set = 0;
	CLRF       _set+0
;dc.c,120 :: 		if(PORTA.F0 == 0)
	BTFSC      PORTA+0, 0
	GOTO       L_main2
;dc.c,122 :: 		Delay_ms(100);
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_main3:
	DECFSZ     R13+0, 1
	GOTO       L_main3
	DECFSZ     R12+0, 1
	GOTO       L_main3
	DECFSZ     R11+0, 1
	GOTO       L_main3
	NOP
;dc.c,123 :: 		if(PORTA.F0 == 0)
	BTFSC      PORTA+0, 0
	GOTO       L_main4
;dc.c,125 :: 		set_count++;
	INCF       _set_count+0, 1
;dc.c,126 :: 		if(set_count >= 7)
	MOVLW      7
	SUBWF      _set_count+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L_main5
;dc.c,128 :: 		set_count = 0;
	CLRF       _set_count+0
;dc.c,129 :: 		}
L_main5:
;dc.c,130 :: 		}
L_main4:
;dc.c,131 :: 		}
L_main2:
;dc.c,132 :: 		if(set_count)
	MOVF       _set_count+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main6
;dc.c,134 :: 		if(PORTA.F1 == 0)
	BTFSC      PORTA+0, 1
	GOTO       L_main7
;dc.c,136 :: 		Delay_ms(100);
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_main8:
	DECFSZ     R13+0, 1
	GOTO       L_main8
	DECFSZ     R12+0, 1
	GOTO       L_main8
	DECFSZ     R11+0, 1
	GOTO       L_main8
	NOP
;dc.c,137 :: 		if(PORTA.F1 == 0)
	BTFSC      PORTA+0, 1
	GOTO       L_main9
;dc.c,138 :: 		set = 1;
	MOVLW      1
	MOVWF      _set+0
L_main9:
;dc.c,139 :: 		}
L_main7:
;dc.c,141 :: 		if(PORTA.F2 == 0)
	BTFSC      PORTA+0, 2
	GOTO       L_main10
;dc.c,143 :: 		Delay_ms(100);
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_main11:
	DECFSZ     R13+0, 1
	GOTO       L_main11
	DECFSZ     R12+0, 1
	GOTO       L_main11
	DECFSZ     R11+0, 1
	GOTO       L_main11
	NOP
;dc.c,144 :: 		if(PORTA.F2 == 0)
	BTFSC      PORTA+0, 2
	GOTO       L_main12
;dc.c,145 :: 		set = -1;
	MOVLW      255
	MOVWF      _set+0
L_main12:
;dc.c,146 :: 		}
L_main10:
;dc.c,147 :: 		if(set_count && set)
	MOVF       _set_count+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main15
	MOVF       _set+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main15
L__main39:
;dc.c,149 :: 		switch(set_count)
	GOTO       L_main16
;dc.c,151 :: 		case 1:
L_main18:
;dc.c,152 :: 		hour = BCD2Binary(hour);
	MOVF       _hour+0, 0
	MOVWF      FARG_BCD2Binary_a+0
	MOVF       _hour+1, 0
	MOVWF      FARG_BCD2Binary_a+1
	CALL       _BCD2Binary+0
	MOVF       R0+0, 0
	MOVWF      _hour+0
	MOVF       R0+1, 0
	MOVWF      _hour+1
;dc.c,153 :: 		hour = hour + set;
	MOVF       _set+0, 0
	ADDWF      R0+0, 1
	BTFSC      STATUS+0, 0
	INCF       R0+1, 1
	MOVLW      0
	BTFSC      _set+0, 7
	MOVLW      255
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _hour+0
	MOVF       R0+1, 0
	MOVWF      _hour+1
;dc.c,154 :: 		hour = Binary2BCD(hour);
	MOVF       R0+0, 0
	MOVWF      FARG_Binary2BCD_a+0
	MOVF       R0+1, 0
	MOVWF      FARG_Binary2BCD_a+1
	CALL       _Binary2BCD+0
	MOVF       R0+0, 0
	MOVWF      _hour+0
	MOVF       R0+1, 0
	MOVWF      _hour+1
;dc.c,155 :: 		if((hour & 0x1F) >= 0x13)
	MOVLW      31
	ANDWF      R0+0, 0
	MOVWF      R2+0
	MOVF       R0+1, 0
	MOVWF      R2+1
	MOVLW      0
	ANDWF      R2+1, 1
	MOVLW      128
	XORWF      R2+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main44
	MOVLW      19
	SUBWF      R2+0, 0
L__main44:
	BTFSS      STATUS+0, 0
	GOTO       L_main19
;dc.c,157 :: 		hour = hour & 0b11100001;
	MOVLW      225
	ANDWF      _hour+0, 1
	MOVLW      0
	ANDWF      _hour+1, 1
;dc.c,158 :: 		hour = hour ^ 0x20;
	MOVLW      32
	XORWF      _hour+0, 1
	MOVLW      0
	XORWF      _hour+1, 1
;dc.c,159 :: 		}
	GOTO       L_main20
L_main19:
;dc.c,160 :: 		else if((hour & 0x1F) <= 0x00)
	MOVLW      31
	ANDWF      _hour+0, 0
	MOVWF      R1+0
	MOVF       _hour+1, 0
	MOVWF      R1+1
	MOVLW      0
	ANDWF      R1+1, 1
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      R1+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main45
	MOVF       R1+0, 0
	SUBLW      0
L__main45:
	BTFSS      STATUS+0, 0
	GOTO       L_main21
;dc.c,162 :: 		hour = hour | 0b00010010;
	MOVLW      18
	IORWF      _hour+0, 1
	MOVLW      0
	IORWF      _hour+1, 1
;dc.c,163 :: 		hour = hour ^ 0x20;
	MOVLW      32
	XORWF      _hour+0, 1
	MOVLW      0
	XORWF      _hour+1, 1
;dc.c,164 :: 		}
L_main21:
L_main20:
;dc.c,165 :: 		write_ds1307(2, hour); //write hour
	MOVLW      2
	MOVWF      FARG_write_ds1307_address+0
	MOVF       _hour+0, 0
	MOVWF      FARG_write_ds1307_w_data+0
	CALL       _write_ds1307+0
;dc.c,166 :: 		break;
	GOTO       L_main17
;dc.c,167 :: 		case 2:
L_main22:
;dc.c,168 :: 		minute = BCD2Binary(minute);
	MOVF       _minute+0, 0
	MOVWF      FARG_BCD2Binary_a+0
	MOVF       _minute+1, 0
	MOVWF      FARG_BCD2Binary_a+1
	CALL       _BCD2Binary+0
	MOVF       R0+0, 0
	MOVWF      _minute+0
	MOVF       R0+1, 0
	MOVWF      _minute+1
;dc.c,169 :: 		minute = minute + set;
	MOVF       _set+0, 0
	ADDWF      R0+0, 0
	MOVWF      R2+0
	MOVF       R0+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	MOVWF      R2+1
	MOVLW      0
	BTFSC      _set+0, 7
	MOVLW      255
	ADDWF      R2+1, 1
	MOVF       R2+0, 0
	MOVWF      _minute+0
	MOVF       R2+1, 0
	MOVWF      _minute+1
;dc.c,170 :: 		if(minute >= 60)
	MOVLW      128
	XORWF      R2+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main46
	MOVLW      60
	SUBWF      R2+0, 0
L__main46:
	BTFSS      STATUS+0, 0
	GOTO       L_main23
;dc.c,171 :: 		minute = 0;
	CLRF       _minute+0
	CLRF       _minute+1
L_main23:
;dc.c,172 :: 		if(minute < 0)
	MOVLW      128
	XORWF      _minute+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main47
	MOVLW      0
	SUBWF      _minute+0, 0
L__main47:
	BTFSC      STATUS+0, 0
	GOTO       L_main24
;dc.c,173 :: 		minute = 59;
	MOVLW      59
	MOVWF      _minute+0
	MOVLW      0
	MOVWF      _minute+1
L_main24:
;dc.c,174 :: 		minute = Binary2BCD(minute);
	MOVF       _minute+0, 0
	MOVWF      FARG_Binary2BCD_a+0
	MOVF       _minute+1, 0
	MOVWF      FARG_Binary2BCD_a+1
	CALL       _Binary2BCD+0
	MOVF       R0+0, 0
	MOVWF      _minute+0
	MOVF       R0+1, 0
	MOVWF      _minute+1
;dc.c,175 :: 		write_ds1307(1, minute); //write min
	MOVLW      1
	MOVWF      FARG_write_ds1307_address+0
	MOVF       R0+0, 0
	MOVWF      FARG_write_ds1307_w_data+0
	CALL       _write_ds1307+0
;dc.c,176 :: 		break;
	GOTO       L_main17
;dc.c,177 :: 		case 3:
L_main25:
;dc.c,178 :: 		if(abs(set))
	MOVF       _set+0, 0
	MOVWF      FARG_abs_a+0
	MOVLW      0
	BTFSC      FARG_abs_a+0, 7
	MOVLW      255
	MOVWF      FARG_abs_a+1
	CALL       _abs+0
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main26
;dc.c,179 :: 		write_ds1307(0,0x00); //Reset second to 0 sec. and start Oscillator
	CLRF       FARG_write_ds1307_address+0
	CLRF       FARG_write_ds1307_w_data+0
	CALL       _write_ds1307+0
L_main26:
;dc.c,180 :: 		break;
	GOTO       L_main17
;dc.c,181 :: 		case 4:
L_main27:
;dc.c,182 :: 		day = BCD2Binary(day);
	MOVF       _day+0, 0
	MOVWF      FARG_BCD2Binary_a+0
	MOVF       _day+1, 0
	MOVWF      FARG_BCD2Binary_a+1
	CALL       _BCD2Binary+0
	MOVF       R0+0, 0
	MOVWF      _day+0
	MOVF       R0+1, 0
	MOVWF      _day+1
;dc.c,183 :: 		day = day + set;
	MOVF       _set+0, 0
	ADDWF      R0+0, 1
	BTFSC      STATUS+0, 0
	INCF       R0+1, 1
	MOVLW      0
	BTFSC      _set+0, 7
	MOVLW      255
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _day+0
	MOVF       R0+1, 0
	MOVWF      _day+1
;dc.c,184 :: 		day = Binary2BCD(day);
	MOVF       R0+0, 0
	MOVWF      FARG_Binary2BCD_a+0
	MOVF       R0+1, 0
	MOVWF      FARG_Binary2BCD_a+1
	CALL       _Binary2BCD+0
	MOVF       R0+0, 0
	MOVWF      _day+0
	MOVF       R0+1, 0
	MOVWF      _day+1
;dc.c,185 :: 		if(day >= 0x32)
	MOVLW      128
	XORWF      R0+1, 0
	MOVWF      R2+0
	MOVLW      128
	SUBWF      R2+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main48
	MOVLW      50
	SUBWF      R0+0, 0
L__main48:
	BTFSS      STATUS+0, 0
	GOTO       L_main28
;dc.c,186 :: 		day = 1;
	MOVLW      1
	MOVWF      _day+0
	MOVLW      0
	MOVWF      _day+1
L_main28:
;dc.c,187 :: 		if(day <= 0)
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      _day+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main49
	MOVF       _day+0, 0
	SUBLW      0
L__main49:
	BTFSS      STATUS+0, 0
	GOTO       L_main29
;dc.c,188 :: 		day = 0x31;
	MOVLW      49
	MOVWF      _day+0
	MOVLW      0
	MOVWF      _day+1
L_main29:
;dc.c,189 :: 		write_ds1307(4, day); // write date 17
	MOVLW      4
	MOVWF      FARG_write_ds1307_address+0
	MOVF       _day+0, 0
	MOVWF      FARG_write_ds1307_w_data+0
	CALL       _write_ds1307+0
;dc.c,190 :: 		break;
	GOTO       L_main17
;dc.c,191 :: 		case 5:
L_main30:
;dc.c,192 :: 		month = BCD2Binary(month);
	MOVF       _month+0, 0
	MOVWF      FARG_BCD2Binary_a+0
	MOVF       _month+1, 0
	MOVWF      FARG_BCD2Binary_a+1
	CALL       _BCD2Binary+0
	MOVF       R0+0, 0
	MOVWF      _month+0
	MOVF       R0+1, 0
	MOVWF      _month+1
;dc.c,193 :: 		month = month + set;
	MOVF       _set+0, 0
	ADDWF      R0+0, 1
	BTFSC      STATUS+0, 0
	INCF       R0+1, 1
	MOVLW      0
	BTFSC      _set+0, 7
	MOVLW      255
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _month+0
	MOVF       R0+1, 0
	MOVWF      _month+1
;dc.c,194 :: 		month = Binary2BCD(month);
	MOVF       R0+0, 0
	MOVWF      FARG_Binary2BCD_a+0
	MOVF       R0+1, 0
	MOVWF      FARG_Binary2BCD_a+1
	CALL       _Binary2BCD+0
	MOVF       R0+0, 0
	MOVWF      _month+0
	MOVF       R0+1, 0
	MOVWF      _month+1
;dc.c,195 :: 		if(month > 0x12)
	MOVLW      128
	MOVWF      R2+0
	MOVLW      128
	XORWF      R0+1, 0
	SUBWF      R2+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main50
	MOVF       R0+0, 0
	SUBLW      18
L__main50:
	BTFSC      STATUS+0, 0
	GOTO       L_main31
;dc.c,196 :: 		month = 1;
	MOVLW      1
	MOVWF      _month+0
	MOVLW      0
	MOVWF      _month+1
L_main31:
;dc.c,197 :: 		if(month <= 0)
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      _month+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main51
	MOVF       _month+0, 0
	SUBLW      0
L__main51:
	BTFSS      STATUS+0, 0
	GOTO       L_main32
;dc.c,198 :: 		month = 0x12;
	MOVLW      18
	MOVWF      _month+0
	MOVLW      0
	MOVWF      _month+1
L_main32:
;dc.c,199 :: 		write_ds1307(5,month); // write month 6 June
	MOVLW      5
	MOVWF      FARG_write_ds1307_address+0
	MOVF       _month+0, 0
	MOVWF      FARG_write_ds1307_w_data+0
	CALL       _write_ds1307+0
;dc.c,200 :: 		break;
	GOTO       L_main17
;dc.c,201 :: 		case 6:
L_main33:
;dc.c,202 :: 		year = BCD2Binary(year);
	MOVF       _year+0, 0
	MOVWF      FARG_BCD2Binary_a+0
	MOVF       _year+1, 0
	MOVWF      FARG_BCD2Binary_a+1
	CALL       _BCD2Binary+0
	MOVF       R0+0, 0
	MOVWF      _year+0
	MOVF       R0+1, 0
	MOVWF      _year+1
;dc.c,203 :: 		year = year + set;
	MOVF       _set+0, 0
	ADDWF      R0+0, 1
	BTFSC      STATUS+0, 0
	INCF       R0+1, 1
	MOVLW      0
	BTFSC      _set+0, 7
	MOVLW      255
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _year+0
	MOVF       R0+1, 0
	MOVWF      _year+1
;dc.c,204 :: 		year = Binary2BCD(year);
	MOVF       R0+0, 0
	MOVWF      FARG_Binary2BCD_a+0
	MOVF       R0+1, 0
	MOVWF      FARG_Binary2BCD_a+1
	CALL       _Binary2BCD+0
	MOVF       R0+0, 0
	MOVWF      _year+0
	MOVF       R0+1, 0
	MOVWF      _year+1
;dc.c,205 :: 		if(year <= -1)
	MOVLW      127
	MOVWF      R2+0
	MOVLW      128
	XORWF      R0+1, 0
	SUBWF      R2+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main52
	MOVF       R0+0, 0
	SUBLW      255
L__main52:
	BTFSS      STATUS+0, 0
	GOTO       L_main34
;dc.c,206 :: 		year = 0x99;
	MOVLW      153
	MOVWF      _year+0
	CLRF       _year+1
L_main34:
;dc.c,207 :: 		if(year >= 0x50)
	MOVLW      128
	XORWF      _year+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main53
	MOVLW      80
	SUBWF      _year+0, 0
L__main53:
	BTFSS      STATUS+0, 0
	GOTO       L_main35
;dc.c,208 :: 		year = 0;
	CLRF       _year+0
	CLRF       _year+1
L_main35:
;dc.c,209 :: 		write_ds1307(6, year); // write year
	MOVLW      6
	MOVWF      FARG_write_ds1307_address+0
	MOVF       _year+0, 0
	MOVWF      FARG_write_ds1307_w_data+0
	CALL       _write_ds1307+0
;dc.c,210 :: 		break;
	GOTO       L_main17
;dc.c,211 :: 		}
L_main16:
	MOVF       _set_count+0, 0
	XORLW      1
	BTFSC      STATUS+0, 2
	GOTO       L_main18
	MOVF       _set_count+0, 0
	XORLW      2
	BTFSC      STATUS+0, 2
	GOTO       L_main22
	MOVF       _set_count+0, 0
	XORLW      3
	BTFSC      STATUS+0, 2
	GOTO       L_main25
	MOVF       _set_count+0, 0
	XORLW      4
	BTFSC      STATUS+0, 2
	GOTO       L_main27
	MOVF       _set_count+0, 0
	XORLW      5
	BTFSC      STATUS+0, 2
	GOTO       L_main30
	MOVF       _set_count+0, 0
	XORLW      6
	BTFSC      STATUS+0, 2
	GOTO       L_main33
L_main17:
;dc.c,212 :: 		}
L_main15:
;dc.c,213 :: 		}
L_main6:
;dc.c,215 :: 		second = read_ds1307(0);
	CLRF       FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _second+0
	CLRF       _second+1
;dc.c,216 :: 		minute = read_ds1307(1);
	MOVLW      1
	MOVWF      FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _minute+0
	CLRF       _minute+1
;dc.c,217 :: 		hour = read_ds1307(2);
	MOVLW      2
	MOVWF      FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _hour+0
	CLRF       _hour+1
;dc.c,218 :: 		hr = hour & 0b00011111;
	MOVLW      31
	ANDWF      _hour+0, 0
	MOVWF      _hr+0
	MOVF       _hour+1, 0
	MOVWF      _hr+1
	MOVLW      0
	ANDWF      _hr+1, 1
;dc.c,219 :: 		ap = hour & 0b00100000;
	MOVLW      32
	ANDWF      _hour+0, 0
	MOVWF      _ap+0
	MOVF       _hour+1, 0
	MOVWF      _ap+1
	MOVLW      0
	ANDWF      _ap+1, 1
;dc.c,220 :: 		dday = read_ds1307(3);
	MOVLW      3
	MOVWF      FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _dday+0
	CLRF       _dday+1
;dc.c,221 :: 		day = read_ds1307(4);
	MOVLW      4
	MOVWF      FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _day+0
	CLRF       _day+1
;dc.c,222 :: 		month = read_ds1307(5);
	MOVLW      5
	MOVWF      FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _month+0
	CLRF       _month+1
;dc.c,223 :: 		year = read_ds1307(6);
	MOVLW      6
	MOVWF      FARG_read_ds1307_address+0
	CALL       _read_ds1307+0
	MOVF       R0+0, 0
	MOVWF      _year+0
	CLRF       _year+1
;dc.c,226 :: 		time[0] = BCD2UpperCh(hr);
	MOVF       _hr+0, 0
	MOVWF      FARG_BCD2UpperCh_bcd+0
	CALL       _BCD2UpperCh+0
	MOVF       R0+0, 0
	MOVWF      _time+0
;dc.c,227 :: 		time[1] = BCD2LowerCh(hr);
	MOVF       _hr+0, 0
	MOVWF      FARG_BCD2LowerCh_bcd+0
	CALL       _BCD2LowerCh+0
	MOVF       R0+0, 0
	MOVWF      _time+1
;dc.c,228 :: 		time[3] = BCD2UpperCh(minute);
	MOVF       _minute+0, 0
	MOVWF      FARG_BCD2UpperCh_bcd+0
	CALL       _BCD2UpperCh+0
	MOVF       R0+0, 0
	MOVWF      _time+3
;dc.c,229 :: 		time[4] = BCD2LowerCh(minute);
	MOVF       _minute+0, 0
	MOVWF      FARG_BCD2LowerCh_bcd+0
	CALL       _BCD2LowerCh+0
	MOVF       R0+0, 0
	MOVWF      _time+4
;dc.c,230 :: 		time[6] = BCD2UpperCh(second);
	MOVF       _second+0, 0
	MOVWF      FARG_BCD2UpperCh_bcd+0
	CALL       _BCD2UpperCh+0
	MOVF       R0+0, 0
	MOVWF      _time+6
;dc.c,231 :: 		time[7] = BCD2LowerCh(second);
	MOVF       _second+0, 0
	MOVWF      FARG_BCD2LowerCh_bcd+0
	CALL       _BCD2LowerCh+0
	MOVF       R0+0, 0
	MOVWF      _time+7
;dc.c,233 :: 		date[0] = BCD2UpperCh(day);
	MOVF       _day+0, 0
	MOVWF      FARG_BCD2UpperCh_bcd+0
	CALL       _BCD2UpperCh+0
	MOVF       R0+0, 0
	MOVWF      _date+0
;dc.c,234 :: 		date[1] = BCD2LowerCh(day);
	MOVF       _day+0, 0
	MOVWF      FARG_BCD2LowerCh_bcd+0
	CALL       _BCD2LowerCh+0
	MOVF       R0+0, 0
	MOVWF      _date+1
;dc.c,235 :: 		date[3] = BCD2UpperCh(month);
	MOVF       _month+0, 0
	MOVWF      FARG_BCD2UpperCh_bcd+0
	CALL       _BCD2UpperCh+0
	MOVF       R0+0, 0
	MOVWF      _date+3
;dc.c,236 :: 		date[4] = BCD2LowerCh(month);
	MOVF       _month+0, 0
	MOVWF      FARG_BCD2LowerCh_bcd+0
	CALL       _BCD2LowerCh+0
	MOVF       R0+0, 0
	MOVWF      _date+4
;dc.c,237 :: 		date[6] = BCD2UpperCh(year);
	MOVF       _year+0, 0
	MOVWF      FARG_BCD2UpperCh_bcd+0
	CALL       _BCD2UpperCh+0
	MOVF       R0+0, 0
	MOVWF      _date+6
;dc.c,238 :: 		date[7] = BCD2LowerCh(year);
	MOVF       _year+0, 0
	MOVWF      FARG_BCD2LowerCh_bcd+0
	CALL       _BCD2LowerCh+0
	MOVF       R0+0, 0
	MOVWF      _date+7
;dc.c,240 :: 		if(ap)
	MOVF       _ap+0, 0
	IORWF      _ap+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main36
;dc.c,242 :: 		time[9] = 'P';
	MOVLW      80
	MOVWF      _time+9
;dc.c,243 :: 		time[10] = 'M';
	MOVLW      77
	MOVWF      _time+10
;dc.c,244 :: 		}
	GOTO       L_main37
L_main36:
;dc.c,247 :: 		time[9] = 'A';
	MOVLW      65
	MOVWF      _time+9
;dc.c,248 :: 		time[10] = 'M';
	MOVLW      77
	MOVWF      _time+10
;dc.c,249 :: 		}
L_main37:
;dc.c,252 :: 		Lcd_out(1, 6, time);
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      6
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      _time+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;dc.c,253 :: 		Lcd_out(2, 6, date);
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      6
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      _date+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;dc.c,254 :: 		Delay_ms(100);
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_main38:
	DECFSZ     R13+0, 1
	GOTO       L_main38
	DECFSZ     R12+0, 1
	GOTO       L_main38
	DECFSZ     R11+0, 1
	GOTO       L_main38
	NOP
;dc.c,257 :: 		}while(1);
	GOTO       L_main0
;dc.c,258 :: 		}
	GOTO       $+0
; end of _main
