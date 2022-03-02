 //Function for reading byte

int ReadByte(String PortSpecifier){
  //Initializations
  //DCB dcb;
  int retVal;
  byte Byte;
  unsigned int dwBytesTransferred;
  unsigned int dwCommModemStatus;
  
  PortSpecifier.c_str();
  
  void* hPort = SDOpen(PortSpecifier, FILE_READ )
  );
  //Current control settings for the comm device
  if (!GetCommState(hPort,&dcb))
    return 0x100;
  dcb.BaudRate = CBR_9600; //9600 Baud
  dcb.ByteSize = 8; //8 data bits
  dcb.Parity = NOPARITY; //no parity
  dcb.StopBits = ONESTOPBIT; //1 stop
  if (!SetCommState(hPort,&dcb))
    return 0x100;
  SetCommMask (hPort, EV_RXCHAR | EV_ERR); //receive character event
  WaitCommEvent (hPort, &dwCommModemStatus, 0); //wait for character
  if (dwCommModemStatus & EV_RXCHAR)
    ReadFile (hPort, &Byte, 1, &dwBytesTransferred, 0); //read 1
  
  else if (dwCommModemStatus & EV_ERR)
  retVal = 0x101;
  retVal = Byte;
  CloseHandle(hPort);
  return retVal;
} 

void setup() {
  // put your setup code here, to run once:
  String xbee_port = 0; //assign the port here
}

void loop() {
  // put your main code here, to run repeatedly:
  ReadByte(xbee_port)
}
