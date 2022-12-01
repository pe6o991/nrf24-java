package com;

import com.pi4j.io.spi.SpiChannel;
import com.pi4j.io.spi.SpiDevice;
import com.pi4j.io.spi.SpiFactory;
import com.pi4j.io.gpio.*;
//import com.pi4j.platform.Platform;
//import com.pi4j.platform.PlatformAlreadyAssignedException;
//import com.pi4j.platform.PlatformManager;

import java.io.IOException;
import java.util.Arrays;

public class nrf24lib{

	//Memory map
	public static final byte CONFIG=0x0;
	public static final byte EN_AA=0x01;
	public static final byte EN_RXADDR=0x02;
	public static final byte SETUP_AW=0x03;
	public static final byte SETUP_RETR=0x04;
	public static final byte RF_CH=0x05;
	public static final byte RF_SETUP=0x06;
	public static final byte STATUS=0x07;
	public static final byte OBSERVE_TX=0x08;
	public static final byte CD=0x09;
	public static final byte RX_ADDR_P0=0x0A;
	public static final byte RX_ADDR_P1=0x0B;
	public static final byte RX_ADDR_P2=0x0C;
	public static final byte RX_ADDR_P3=0x0D;
	public static final byte RX_ADDR_P4=0x0E;
	public static final byte RX_ADDR_P5=0x0F;
	public static final byte TX_ADDR=0x10;
	public static final byte RX_PW_P0=0x11;
	public static final byte RX_PW_P1=0x12;
	public static final byte RX_PW_P2=0x13;
	public static final byte RX_PW_P3=0x14;
	public static final byte RX_PW_P4=0x15;
	public static final byte RX_PW_P5=0x16;
	public static final byte FIFO_STATUS=0x17;
	public static final byte DYNPD=0x1C;
	public static final byte FEATURE=0x1D;

	//Instructions
	public static final byte R_REGISTER = 0x00;
	public static final byte W_REGISTER = 0x20;
	public static final byte REGISTER_MASK = 0x1F;
	public static final byte ACTIVATE = 0x50;
	public static final byte R_RX_PL_WID = 0x60;
	public static final byte R_RX_PAYLOAD = 0x61;
	public static final byte W_TX_PAYLOAD = (byte)0xA0;
	public static final byte W_ACK_PAYLOAD = (byte)0xA8;
	public static final byte FLUSH_TX = (byte)0xE1;
	public static final byte FLUSH_RX = (byte)0xE2;
	public static final byte REUSE_TX_PL = (byte)0xE3;
	public static final byte NOP = (byte)0xFF;

	//P model stuff
	public static final byte RPD=0x09;
	public static final byte RF_DR_LOW=5;
	public static final byte RF_DR_HIGH=3;
	public static final byte RF_PWR_LOW=1;
	public static final byte RF_PWR_HIGH=2;

	/* Bit Mnemonics */
	public static final byte MASK_RX_DR=6;
	public static final byte MASK_TX_DS=5;
	public static final byte MASK_MAX_RT=4;
	public static final byte EN_CRC=3;
	public static final byte CRCO=2;
	public static final byte PWR_UP=1;
	public static final byte PRIM_RX=0;
	public static final byte ENAA_P5=5;
	public static final byte ENAA_P4=4;
	public static final byte ENAA_P3=3;
	public static final byte ENAA_P2=2;
	public static final byte ENAA_P1=1;
	public static final byte ENAA_P0=0;
	public static final byte ERX_P5=5;
	public static final byte ERX_P4=4;
	public static final byte ERX_P3=3;
	public static final byte ERX_P2=2;
	public static final byte ERX_P1=1;
	public static final byte ERX_P0=0;
	public static final byte AW=0;
	public static final byte ARD=4;
	public static final byte ARC=0;
	public static final byte PLL_LOCK=4;
	public static final byte RF_DR=3;
	public static final byte RF_PWR=6;
	public static final byte RX_DR=6;
	public static final byte TX_DS=5;
	public static final byte MAX_RT=4;
	public static final byte RX_P_NO=1;
	public static final byte TX_FULL=0;
	public static final byte PLOS_CNT=4;
	public static final byte ARC_CNT=0;
	public static final byte TX_REUSE=6;
	public static final byte FIFO_FULL=5;
	public static final byte TX_EMPTY=4;
	public static final byte RX_FULL=1;
	public static final byte RX_EMPTY=0;
	public static final byte DPL_P5=5;
	public static final byte DPL_P4=4;
	public static final byte DPL_P3=3;
	public static final byte DPL_P2=2;
	public static final byte DPL_P1=1;
	public static final byte DPL_P0=0;
	public static final byte EN_DPL=2;
	public static final byte EN_ACK_PAY=1;
	public static final byte EN_DYN_ACK=0;

	//Random defines for the class
	public static final byte LOW = 0;
	public static final byte HIGH=1;
	public static final byte RF24_250KBPS=1;
	public static final byte RF24_1MBPS=2;
	public static final byte RF24_2MBPS=3;
	public static final byte RF24_PA_MIN=1;
	public static final byte RF24_PA_LOW=2;
	public static final byte RF24_PA_HIGH=3;
	public static final byte RF24_PA_MAX=4;
	public static final byte RF24_PA_ERROR=5;
	public static final byte RF24_CRC_DISABLED=0;
	public static final byte RF24_CRC_8=1;
	public static final byte RF24_CRC_16=2;


	private static SpiDevice spi = null;
	private static final GpioController gpio = GpioFactory.getInstance();
	private static GpioPinDigitalOutput outputPin;
	private static Pin pins[] = RaspiPin.allPins();
	private byte[] pipe0_reading_address = new byte[5];
	private byte payload_size;
	private byte ack_payload_length;
	private boolean p_variant;
	private boolean wide_band;
	private boolean ack_payload_available;
	private boolean dynamic_payloads_enabled;

	private static final byte[] child_pipe = {RX_ADDR_P0,RX_ADDR_P1,RX_ADDR_P2,RX_ADDR_P3,RX_ADDR_P4,RX_ADDR_P5};
	private static final byte[] child_payload_size = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};
	private static final byte[] child_pipe_enable = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

	public nrf24lib(){
		Arrays.sort(pins);
		try{
		spi = SpiFactory.getInstance(SpiChannel.CS0,SpiDevice.DEFAULT_SPI_SPEED,SpiDevice.DEFAULT_SPI_MODE);
		}catch(IOException e){
			e.printStackTrace();
		}
		outputPin=gpio.provisionDigitalOutputPin(pins[6],PinState.LOW);
		wide_band=true;
		p_variant=false;
		payload_size=32;
		ack_payload_available=false;
		dynamic_payloads_enabled=false;
		for(int i=0;i<pipe0_reading_address.length;i++)
				pipe0_reading_address[i]=(byte)0;

	}

	public byte BV(byte num){
		byte val=0;
		switch(num){
			case 0: val=1; break;
			case 1: val=2; break;
			case 2: val=4; break;
			case 3: val=8; break;
			case 4: val=16; break;
			case 5: val=32; break;
			case 6: val=64; break;
			case 7: val=(byte)128; break;
		}
		return val;
	}
/*******************************************************************/
	public void ce(byte val){
		if(val==0)
			outputPin.low();		
		else
			outputPin.high();
	}
/*******************************************************************/
	public byte[] read_register(byte register,int size){
		byte[] data = new byte[size+1];
		for(int i=0;i<size+1;i++){
			data[i]=(byte)0xff;
		}
		data[0]=(byte)(R_REGISTER | register);
		byte[] result=new byte[size+1];
		byte[] ret = new byte[size];
		try{
			result=spi.write(data,0,size+1);
		}catch(IOException e){
			e.printStackTrace();
		}
		for(int i=1;i<result.length;i++){
			ret[i-1]=result[i];
		}
		return ret;
	}
/*******************************************************************/
    public byte read_register(byte register){
   	byte data[] = new byte[]{(byte) (R_REGISTER | register),0};
	try{
       		byte[] result = spi.write(data);
		return result[1];
	}catch(IOException e){
		e.printStackTrace();
	}
	return 0;	
    }
/*******************************************************************/
	public void write_register(byte register,byte[] buf,int size){
		byte data[]=new byte[size+1];
		data[0]=(byte)(W_REGISTER | register);
		for(int i=1;i<size+1;i++){
			data[i]=buf[i-1];
		}
		try{
			spi.write(data);
		}catch(IOException e){ e.printStackTrace(); }
	}
/********************************************************************/
	public void write_register(byte register,int value){
		byte data[] = new byte[]{(byte) (W_REGISTER | register),(byte)value};
		try{
			spi.write(data);
		}catch(IOException e){ e.printStackTrace(); }
	}
/***********************************************************************/
	public void write_payload(byte[] buf,int size){
		byte data[]=new byte[size+1];
		data[0]=W_TX_PAYLOAD;
		for(int i=1;i<size+1;i++){
			data[i]=buf[i-1];
		}
		try{
			spi.write(data);
		}catch(IOException e){ e.printStackTrace(); }
	}
/***********************************************************************/
	public byte[] read_payload(int size){
		byte[] data = new byte[size+1];
		for(int i=0;i<size+1;i++) data[i]=(byte)0xff;
		data[0]=R_RX_PAYLOAD;
		byte[] result=new byte[size+1];
		try{
			result=spi.write(data,0,size+1);
		}catch(IOException e){ e.printStackTrace(); }
		for(int i=1;i<result.length;i++){
			data[i-1]=result[i];
		}
		return data;
	}
/***********************************************************************/
	public void flush_rx() throws IOException{
		spi.write(FLUSH_RX);
	}
/***********************************************************************/
	public void flush_tx() throws IOException{
		spi.write(FLUSH_TX);
	}
/***********************************************************************/
	public byte get_status(){
		byte[] data=new byte[2];
		try{
		data = spi.write(NOP);	
		}catch(IOException e){ e.printStackTrace(); }
		return data[0];
	}
/***********************************************************************/
	public void setChannel(int channel){
		byte max_channel = 127;
		if(channel > max_channel){
			write_register(RF_CH,max_channel);
		}else{
			write_register(RF_CH,channel);
		}
	}
/**********************************************************************/
	public void setPayloadSize(byte size){
		byte max_payload_size = 32;
		if(size > max_payload_size){
			payload_size = max_payload_size;
		}else{
			payload_size = size;
		}
	}
/***********************************************************************/
	public byte getPayloadSize(){
		return payload_size;
	}
/***********************************************************************/
	public void begin() throws IOException{
		ce(LOW);
		write_register(SETUP_RETR,(4 << ARD) | (15 << ARC));
		setPALevel(RF24_PA_MAX);
		if(setDataRate(RF24_250KBPS)){
			p_variant = true;
		}
		setDataRate(RF24_1MBPS);
		setCRCLength(16);
		write_register(DYNPD,0);
		write_register(STATUS,BV(RX_DR) | BV(TX_DS) | BV(MAX_RT));
		setChannel(76);
		flush_rx();
		flush_tx();
	}
/***********************************************************************/
	public void startListening() throws IOException{
		write_register(CONFIG, read_register(CONFIG) | BV(PWR_UP) | BV(PRIM_RX));
		write_register(STATUS, BV(RX_DR) | BV(TX_DS) | BV(MAX_RT));

		if (pipe0_reading_address[0]!=0)
				write_register(RX_ADDR_P0,pipe0_reading_address, 5);
		flush_rx();
		flush_tx();
		ce(HIGH);
	}
/************************************************************************/
	public void stopListening() throws IOException{
		ce(LOW);
		flush_tx();
		flush_rx();
	}
/*************************************************************************/
	public void powerDown(){
		write_register(CONFIG,read_register(CONFIG) & ~BV(PWR_UP));
	}
/**************************************************************************/
	public void powerUp(){
		write_register(CONFIG,read_register(CONFIG) | BV(PWR_UP));
	}
/**************************************************************************/
	public boolean write(byte[] data,int size) throws IOException {
		boolean result=false;
		byte status;
		int count=0;
		startwrite(data,size);
		do{
			status=read_register(STATUS);
			count++;
		}while((status & BV(TX_DS))==0 && (status & BV(MAX_RT))==0 && count < 5);

		if((get_status() & BV(TX_DS))==BV(TX_DS)) result = true;

		if(ack_payload_available){
			ack_payload_length = getDynamicPayloadSize();
		}

		powerDown();
		flush_tx();
		return result;
	}
/**************************************************************************/
	public void startwrite(byte[] data,int size){
		// Transmitter power-up
		write_register(CONFIG, (read_register(CONFIG) | BV(PWR_UP)) & ~BV(PRIM_RX));
		try{
		Thread.sleep(50);
		// Send the payload
		write_payload(data,size);

		// Allons!
		ce(HIGH);
		Thread.sleep(50);
		ce(LOW);
		}catch(InterruptedException e){ e.printStackTrace(); }
	}
/**************************************************************************/
	public byte getDynamicPayloadSize() throws IOException{
		byte[] data={R_RX_PL_WID,(byte)0xff};
		byte[] result = spi.write(data);
		return result[1];
	}
/**************************************************************************/
	public boolean available(){
		byte status = get_status();
		if((status & BV(RX_DR))==BV(RX_DR)){
			write_register(STATUS,BV(RX_DR));
			write_register(STATUS,BV(TX_DS));
			return true;
		}else{
			return false;
		}
	}
/**************************************************************************/
	public int available(int pipenum) throws IOException{
		byte status = get_status();
		int pipe=7;
		int result = (status & BV(RX_DR));
		if(result==BV(RX_DR)){
			if(pipenum < 6)
				pipe = (status >> RX_P_NO) & 7;

			write_register(STATUS,BV(RX_DR));
			if ((status & BV(TX_DS))==BV(TX_DS)){
				write_register(STATUS,BV(TX_DS));
			}
		}
		return pipe;
	}
/**************************************************************************/
	public byte[] read(int size){
		return read_payload(size);
	}
/**************************************************************************/
	public void openWritingPipe(byte[] address){
		write_register(RX_ADDR_P0,address, 5);
		write_register(TX_ADDR,address, 5);
		write_register(RX_PW_P0,payload_size);
	}
/**************************************************************************/
	public void openReadingPipe(int child, byte[] address){
		if (child == 0)
			pipe0_reading_address = address;
		
		if (child <= 6){
		// For pipes 2-5, only write the LSB
			if ( child < 2 )
				write_register(child_pipe[child],address, 5);
			else
				write_register(child_pipe[child],address[address.length-1]);

			write_register(child_payload_size[child],payload_size);
			// Note it would be more efficient to set all of the bits for all open
			// pipes at once.  However, I thought it would make the calling code
			// more simple to do it this way.
			write_register(EN_RXADDR,read_register(EN_RXADDR) | BV(child_pipe_enable[child]));
		}
	}
/***************************************************************************/
	public void toggle_features(){
		byte data[] = {ACTIVATE,0x73};
		try{
			spi.write(data);
		}catch(IOException e){
			e.printStackTrace();
		}
	}
/****************************************************************************/
	public void enableDynamicPayloads(){

	}
/*****************************************************************************/
	public void enableAckPayload(){

	}
/*****************************************************************************/
	public void writeAckPayload(int pipe, byte[] buf, int len){
		if(len<=32){
			write_register((byte)(W_ACK_PAYLOAD |(pipe & 7)),buf,len);
		}else{
			write_register((byte)(W_ACK_PAYLOAD |(pipe & 7)),buf,32);
		}
	}
/*****************************************************************************/
	public boolean isAckPayloadAvailable(){
		boolean result = ack_payload_available;
		ack_payload_available = false;
		return result;
	}
/******************************************************************************/
	public boolean isPVariant(){
		return p_variant;
	}
/******************************************************************************/
	public void setAutoAck(boolean enable){
		if(enable)
			write_register(EN_AA, 127);
		else
			write_register(EN_AA, 0);
	}
/******************************************************************************/
	public void setAutoAck(int pipe, boolean enable){
		if(pipe <= 6){
			byte en_aa = read_register(EN_AA);
			if(enable){
				en_aa |= BV((byte)pipe) ;
			}else{
				en_aa &= ~BV((byte)pipe) ;
			}
			write_register(EN_AA, en_aa);
		}
	}
/*******************************************************************************/
	public boolean testCarrier(){
		if((read_register(CD) & (byte)1)==1)
			return true;
		else
			return false;
	}
/*******************************************************************************/
	public boolean testRPD(){
		if((read_register(RPD) & (byte)1)==1)
			return true;
		else
			return false;
	}
/*******************************************************************************/
	public void setPALevel(byte level){
		byte setup = read_register(RF_SETUP) ;
		setup &= ~(BV(RF_PWR_LOW) | BV(RF_PWR_HIGH)) ;

		if(level == RF24_PA_MAX){
			setup |= (BV(RF_PWR_LOW) | BV(RF_PWR_HIGH)) ;
		}else if (level == RF24_PA_HIGH){
			setup |= BV(RF_PWR_HIGH) ;
		}else if (level == RF24_PA_LOW){
			setup |= BV(RF_PWR_LOW);
		}else if (level == RF24_PA_MIN){
		   
		}else if (level == RF24_PA_ERROR){
			// On error, go to maximum PA
			setup |= (BV(RF_PWR_LOW) | BV(RF_PWR_HIGH)) ;
		}
		write_register(RF_SETUP, setup);
	}
/********************************************************************************/
	public byte getPALevel(){
		byte result = RF24_PA_ERROR;
		int power =read_register(RF_SETUP) & (BV(RF_PWR_LOW) | BV(RF_PWR_HIGH)) ;
		// switch uses RAM (evil!)
		if (power == (BV(RF_PWR_LOW) | BV(RF_PWR_HIGH))){
			result = RF24_PA_MAX ;
		}else if (power == BV(RF_PWR_HIGH)){
			result = RF24_PA_HIGH;
		}else if (power == BV(RF_PWR_LOW)){
			result = RF24_PA_LOW;
		}else{
			result = RF24_PA_MIN;
		}
		return result;
	}
/**************************************************************************************/
	public boolean setDataRate(byte rate){
		boolean result=false;
		byte setup = read_register(RF_SETUP);
		setup &= ~(BV(RF_DR_LOW) | BV(RF_DR_HIGH));
		if(rate==RF24_250KBPS){
			setup |= BV(RF_DR_LOW);
		}else{
			if(rate==RF24_2MBPS){
				setup |=BV(RF_DR_HIGH);
			}else{
			
			}
		}
		write_register(RF_SETUP,setup);
		if(read_register(RF_SETUP)==setup){
			result=true;
		}
		return result;
	}
/*******************************************************************************************/
	public byte getDataRate(){
		byte result;
		int dr =read_register(RF_SETUP) & (BV(RF_DR_LOW) | BV(RF_DR_HIGH));
		// switch uses RAM (evil!)
		// Order matters in our case below
		if(dr == BV(RF_DR_LOW)){
			// '10' = 250KBPS
			result = RF24_250KBPS;
		}else if(dr == BV(RF_DR_HIGH)){
			// '01' = 2MBPS
			result = RF24_2MBPS;
		}else{
			// '00' = 1MBPS
			result = RF24_1MBPS;
		}
		return result;
	}
/*******************************************************************************************/
	public void setCRCLength(int crc){
		int config = read_register(CONFIG) & ~(BV(CRCO) | BV(EN_CRC));
		// switch uses RAM (evil!)
		if(crc == RF24_CRC_DISABLED){
			// Do nothing, we turned it off above. 
		}else if (crc == RF24_CRC_8){
			config |= BV(EN_CRC);
		}else{
			config |= BV(EN_CRC);
			config |= BV(CRCO);
		}
		write_register(CONFIG,config);
	}
/********************************************************************************************/
	public byte getCRCLength(){
		byte result = RF24_CRC_DISABLED;
		byte config = read_register(CONFIG);
		if((config & BV(EN_CRC))==BV(EN_CRC)){
			if ((config & BV(CRCO))==BV(CRCO))
				result = RF24_CRC_16;
			else 
				result = RF24_CRC_8;
		}
		   return result;
	}
/*******************************************************************************************/
	public void disableCRC(){
		int disable = read_register(CONFIG) & ~BV(EN_CRC);
		write_register(CONFIG,disable);
	}
/*******************************************************************************************/
	public void setRetries(char delay, char count){
		write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
	}
}
