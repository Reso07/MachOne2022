 //Function for reading byte
int ReadByte(CString PortSpecifier)
{
 DCB dcb;
 int retVal;
 BYTE Byte;
 DWORD dwBytesTransferred;
 DWORD dwCommModemStatus;
 HANDLE hPort = CreateFile(

 PortSpecifier,
 GENERIC_READ,
 0,
 NULL,
 OPEN_EXISTING,
 0,
 NULL
 );
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
 serial.println(RetVal);
 return retVal;
}
