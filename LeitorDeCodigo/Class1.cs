using System;
using System.Configuration;
using System.Data;
using System.IO;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using FT_HANDLE = System.UInt32;

namespace CCB.RS232

{

    /// <summary>

    /// Delegate utilizada para comunicacao serial com a central.

    /// </summary>

    public delegate void RS232ReceivedEventHandler(object sender,

    int EventoModulo,

    int EventoNumero,

    int EventoTipo,

    int EventoID,

    DateTime EventoDataInicial,

    DateTime EventoDataFinal,

    int EventoValor,

    string EventoMensagem,

    double EventoTempo);

    /// <summary>

    /// Classe utilizada para comunicacao serial com a central.

    /// </summary>

    public class clsControladorRS232 : IDisposable

    {

        #region >>> FTD2XX >>>

        enum FT_STATUS //:Uint32

        {

            FT_OK = 0,

            FT_INVALID_HANDLE,

            FT_DEVICE_NOT_FOUND,

            FT_DEVICE_NOT_OPENED,

            FT_IO_ERROR,

            FT_INSUFFICIENT_RESOURCES,

            FT_INVALID_PARAMETER,

            FT_INVALID_BAUD_RATE,

            FT_DEVICE_NOT_OPENED_FOR_ERASE,

            FT_DEVICE_NOT_OPENED_FOR_WRITE,

            FT_FAILED_TO_WRITE_DEVICE,

            FT_EEPROM_READ_FAILED,

            FT_EEPROM_WRITE_FAILED,

            FT_EEPROM_ERASE_FAILED,

            FT_EEPROM_NOT_PRESENT,

            FT_EEPROM_NOT_PROGRAMMED,

            FT_INVALID_ARGS,

            FT_OTHER_ERROR

        };

        public const UInt32 FT_BAUD_300 = 300;

        public const UInt32 FT_BAUD_600 = 600;

        public const UInt32 FT_BAUD_1200 = 1200;

        public const UInt32 FT_BAUD_2400 = 2400;

        public const UInt32 FT_BAUD_4800 = 4800;

        public const UInt32 FT_BAUD_9600 = 9600;

        public const UInt32 FT_BAUD_14400 = 14400;

        public const UInt32 FT_BAUD_19200 = 19200;

        public const UInt32 FT_BAUD_38400 = 38400;

        public const UInt32 FT_BAUD_57600 = 57600;

        public const UInt32 FT_BAUD_115200 = 115200;

        public const UInt32 FT_BAUD_230400 = 230400;

        public const UInt32 FT_BAUD_460800 = 460800;

        public const UInt32 FT_BAUD_921600 = 921600;

        public const UInt32 FT_LIST_NUMBER_ONLY = 0x80000000;

        public const UInt32 FT_LIST_BY_INDEX = 0x40000000;

        public const UInt32 FT_LIST_ALL = 0x20000000;

        public const UInt32 FT_OPEN_BY_SERIAL_NUMBER = 1;

        public const UInt32 FT_OPEN_BY_DESCRIPTION = 2;

        // Word Lengths

        public const byte FT_BITS_8 = 8;

        public const byte FT_BITS_7 = 7;

        public const byte FT_BITS_6 = 6;

        public const byte FT_BITS_5 = 5;

        // Stop Bits

        public const byte FT_STOP_BITS_1 = 0;

        public const byte FT_STOP_BITS_1_5 = 1;

        public const byte FT_STOP_BITS_2 = 2;

        // Parity

        public const byte FT_PARITY_NONE = 0;

        public const byte FT_PARITY_ODD = 1;

        public const byte FT_PARITY_EVEN = 2;

        public const byte FT_PARITY_MARK = 3;

        public const byte FT_PARITY_SPACE = 4;

        // Flow Control

        public const UInt16 FT_FLOW_NONE = 0;

        public const UInt16 FT_FLOW_RTS_CTS = 0x0100;

        public const UInt16 FT_FLOW_DTR_DSR = 0x0200;

        public const UInt16 FT_FLOW_XON_XOFF = 0x0400;

        // Purge rx and tx buffers

        public const byte FT_PURGE_RX = 1;

        public const byte FT_PURGE_TX = 2;

        // Events

        public const UInt32 FT_EVENT_RXCHAR = 1;

        public const UInt32 FT_EVENT_MODEM_STATUS = 2;

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_ListDevices(void* pvArg1, void* pvArg2, FT_HANDLE dwFlags); // FT_ListDevices by number only

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_ListDevices(UInt32 pvArg1, void* pvArg2, UInt32 dwFlags); // FT_ListDevcies by serial number or description by index only

        [DllImport("FTD2XX.dll")]

        static extern FT_STATUS FT_Open(UInt32 uiPort, ref FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_OpenEx(void* pvArg1, UInt32 dwFlags, ref FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern FT_STATUS FT_Close(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_Read(FT_HANDLE ftHandle, void* lpBuffer, UInt32 dwBytesToRead, ref UInt32 lpdwBytesReturned);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_Write(FT_HANDLE ftHandle, void* lpBuffer, UInt32 dwBytesToRead, ref UInt32 lpdwBytesWritten);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetBaudRate(FT_HANDLE ftHandle, UInt32 dwBaudRate);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetDataCharacteristics(FT_HANDLE ftHandle, byte uWordLength, byte uStopBits, byte uParity);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetFlowControl(FT_HANDLE ftHandle, char usFlowControl, byte uXon, byte uXoff);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetDtr(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_ClrDtr(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetRts(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_ClrRts(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_GetModemStatus(FT_HANDLE ftHandle, ref UInt32 lpdwModemStatus);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetChars(FT_HANDLE ftHandle, byte uEventCh, byte uEventChEn, byte uErrorCh, byte uErrorChEn);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_Purge(FT_HANDLE ftHandle, UInt32 dwMask);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetTimeouts(FT_HANDLE ftHandle, UInt32 dwReadTimeout, UInt32 dwWriteTimeout);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_GetQueueStatus(FT_HANDLE ftHandle, ref UInt32 lpdwAmountInRxQueue);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetBreakOn(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetBreakOff(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_GetStatus(FT_HANDLE ftHandle, ref UInt32 lpdwAmountInRxQueue, ref UInt32 lpdwAmountInTxQueue, ref UInt32 lpdwEventStatus);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetEventNotification(FT_HANDLE ftHandle, UInt32 dwEventMask, void* pvArg);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_ResetDevice(FT_HANDLE ftHandle);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetDivisor(FT_HANDLE ftHandle, char usDivisor);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_GetLatencyTimer(FT_HANDLE ftHandle, ref byte pucTimer);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetLatencyTimer(FT_HANDLE ftHandle, byte ucTimer);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_GetBitMode(FT_HANDLE ftHandle, ref byte pucMode);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetBitMode(FT_HANDLE ftHandle, byte ucMask, byte ucEnable);

        [DllImport("FTD2XX.dll")]

        static extern unsafe FT_STATUS FT_SetUSBParameters(FT_HANDLE ftHandle, UInt32 dwInTransferSize, UInt32 dwOutTransferSize);

        protected UInt32 dwListDescFlags;

        protected UInt32 m_hPort;

        protected Thread pThreadRead;

        protected Thread pThreadWrite;

        protected bool fContinue;

        #endregion

        #region >>> Atributos <<<

        private clsControladorProduto ControladorProduto;

        public event RS232ReceivedEventHandler OnRS232ReceviedHandler;

        private bool mUSB = true;

        //

        private int Firmware = 440;

        private bool bFlux40 = false;

        private bool bFluxometrosConectados = false;

        //

        private const int codEscape = 255;

        private const int codStart = 250;

        private const int codStop = 251;

        //

        private const byte cmdGet = 0x00;

        private const byte cmdAck = 0x00;

        private const byte cmdNack = 0x29;

        //

        private const byte cmdGetInfo = 0x01;

        private const byte cmdInfo = 0x81;

        private const byte cmdGetPowerInfo = 0x02;

        private const byte cmdPowerInfo = 0x82;

        private const byte cmdGetDateTime = 0x03;

        private const byte cmdDateTime = 0x83;

        private const byte cmdSetDateTime = 0x43;

        private const byte cmdAckSetDateTime = 0xC3;

        private const byte cmdGetActive = 0x04;

        private const byte cmdActive = 0x84;

        private const byte cmdSetActive = 0x44;

        private const byte cmdAckSetActive = 0xC4;

        private const byte cmdGetThresholdPulse = 0x05;

        private const byte cmdThresholdPulse = 0x85;

        private const byte cmdSetThresholdPulse = 0x45;

        private const byte cmdAckSetThresholdPulse = 0xC5;

        private const byte cmdGetThresholdBat = 0x09;

        private const byte cmdThresholdBat = 0x89;

        private const byte cmdSetThresholdBat = 0x46;

        private const byte cmdAckSetThresholdBat = 0xC6;

        private const byte cmdGetFluxCount = 0x07;

        private const byte cmdFluxCount = 0x87;

        private const byte cmdGetFluxSerial = 0x08;

        private const byte cmdFluxSerial = 0x88;

        private const byte cmdGetData = 0x10;

        private const byte cmdFluxConnectStatus = 0xA0;

        private const byte cmdAckFluxConnectStatus = 0x20;

        private const byte cmdFluxBadPulses = 0xA1;

        private const byte cmdAckFluxBadPulses = 0x21;

        private const byte cmdFluxGoodPulses = 0xA2;

        private const byte cmdAckFluxGoodPulses = 0x22;

        private const byte cmdFluxBadPulsesLog = 0xA3;

        private const byte cmdAckFluxBadPulsesLog = 0x23;

        private const byte cmdError = 0x80;

        private const byte cmdAckError = 0x30;

        private const byte cmdResetCentral = 0x2A;

        //

        private byte[] packetRcvBuffer;

        private byte[] packetXMitBuffer;

        //

        private const int MAX_DATA_LENGTH = 3840;

        private const int MAX_BUFFER_SIZE = 32;

        //

        protected byte[] fBufferIn;

        protected byte[] fBufferOut;

        protected byte[] fBufferRead;

        protected int fPosBufferIn;

        protected int fTamPacoteIn;

        protected int fTamBufferOut;

        protected int fTamBufferRead;

        protected int fTimeout;

        protected int fStatus;

        //

        private bool fEscapeado;

        private bool fStart;

        private bool _timedOut;

        private Boolean disposed = false;

        //

        private SerialPort mSerialPort;

        private string mSerialPortCom;

        private bool mSerialPortStop;

        //

        private int mFluxCount;

        private int[] mFluxSerial;

        private int[] mFluxAtivo;

        private int mCentralSerial;

        private int mCentralVersao;

        //

        private StreamWriter fLog;

        private string FileName = string.Empty;

        private bool mLogRS232;

        private string ClienteCodigo;

        private string ServiceDebug;

        //

        private bool mOnGetData;

        //

        private bool mEvento;

        private int mEventoModulo;

        private int mEventoNumero;

        private int mEventoTipo;

        private int mEventoID;

        private DateTime mEventoDataInicial;

        private DateTime mEventoDataFinal;

        private TimeSpan mEventoDT;

        private DateTime mEventoDTI;

        private DateTime mEventoDTF;

        private DateTime mEventoDTA;

        private DateTime mEventoDTS;

        private int mEventoValor;

        private int mEventoIndiceFlux;

        private int mEventoStatusFlux;

        private string mEventoMensagem;

        private double mEventoTempo;

        //

        private int mBaudRate;

        //

        private double RTCAnterior = 0;

        private double RTCTempo = 0;

        private long dti = 0;

        private long dtf = 0;

        private long vlr = 0;

        private long vlrcrc = 0;

        private ushort CRC_SEED = 0xFFFF;

        private ushort calculatedCRC = 0;

        //

        private int mCCBFluxometroID;

        private long mCCBDti;

        private bool mCCBGravar;

        //

        private string mDeviceName;

        #endregion

        #region <<< Construtor <<<

        public clsControladorRS232(string pPorta, bool pLogRS232)

        {

            try

            {

                ControladorProduto = new clsControladorProduto();

                mBaudRate = Convert.ToInt32(ConfigurationSettings.AppSettings["BaudRateCommunication"]);

                fBufferIn = new byte[MAX_BUFFER_SIZE];

                fBufferOut = new byte[MAX_BUFFER_SIZE];

                fBufferRead = new byte[MAX_DATA_LENGTH];

                packetRcvBuffer = new byte[MAX_BUFFER_SIZE];

                packetXMitBuffer = new byte[MAX_BUFFER_SIZE];

                fPosBufferIn = 0;

                fTamPacoteIn = 0;

                fTamBufferOut = 0;

                fTamBufferRead = 0;

                fTimeout = 0;

                fStatus = 0;

                fEscapeado = false;

                fStart = false;

                _timedOut = false;

                mOnGetData = false;

                mLogRS232 = pLogRS232;

                mSerialPortCom = pPorta;

                mSerialPortStop = false;

                if (pPorta == "USB")

                {

                    mUSB = true;

                }

                else

                {

                    mUSB = false;

                }

                mEventoModulo = 0;

                mEventoNumero = 0;

                mEventoTipo = 0;

                mEventoID = 0;

                mEventoDataInicial = DateTime.Now;

                mEventoDataFinal = DateTime.Now;

                mEventoValor = 0;

                mFluxCount = 0;

                mFluxSerial = new int[200];

                mFluxAtivo = new int[200];

                mEvento = false;

                mCentralSerial = 0;

                mCentralVersao = 0;

                mEventoDTI = DateTime.Now;

                mEventoDTF = DateTime.Now;

                mEventoDTA = mEventoDTI;

                mEventoDTS = DateTime.Now;

                mEventoDT = mEventoDTA.Subtract(mEventoDTI);

                mEventoIndiceFlux = 0;

                mEventoStatusFlux = 0;

                mEventoMensagem = string.Empty;

                bFluxometrosConectados = false;

                dti = 0;

                dtf = 0;

                vlr = 0;

                mCCBFluxometroID = 0;

                mCCBDti = 0;

                mCCBGravar = false;

                mDeviceName = string.Empty;

                m_hPort = 0;

                mEventoTempo = 0;

                fContinue = false;

                dwListDescFlags = FT_LIST_BY_INDEX | FT_OPEN_BY_DESCRIPTION;

                ClienteCodigo = ConfigurationSettings.AppSettings["CustomerCode"];

                ServiceDebug = ConfigurationSettings.AppSettings["ServiceDebug_Y_N"];

                FileName = PesquisarDiretorioChoppControl() + "\\Chopp Control 2007\\Logs\\CCBRS232" + ClienteCodigo + ".LOG";

                if (File.Exists(FileName) == false)

                {

                    fLog = File.CreateText(FileName);

                    fLog.AutoFlush = true;

                    fLog.WriteLine("CCB.RS232 - Versão 2.1.1 - Log de Eventos - " + DateTime.Now.ToString());

                }

                else

                {

                    fLog = File.AppendText(FileName);

                    fLog.AutoFlush = true;

                    fLog.WriteLine("CCB.RS232 - Versão 2.1.1 - Log de Eventos - " + DateTime.Now.ToString());

                }

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: clsControladorRS232() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region <<< Destrutor <<<

        // Public IDisposable.Dispose implementation - calls the internal helper,

        public void Dispose()

        {

            Dispose(true);

        }

        // This is "friendly" wrapper method for IDisposable.Dispose

        public void Close()

        {

            // Call the internal diposal helper

            Dispose(true);

        }

        /*

        * This class wraps a SerialPort instance that is not exposed

        * directly to the consuming application. The SerialPort implements

        * IDisposable, and calling Dispose there causes the SafeHandle

        * given by the OS to the serial port to be released.

        *

        * disposing is true if Close or Dispose have been called explicitly,

        * at a point when all managed references are still valid.

        * Since this class does not directly refer to any system resources, and

        * managed resources can only be cleaned up when the refs are valid, we

        * only clean-up if explicitly called.

        *

        * Proper usage of this class is to call Close() as soon as the port isnt

        * needed anymore (as in MainForm.cs). However, if Dispose isnt called here,

        * then the automatic Dispose call from finalization of the SerialPort

        * itself will free the OS handle to the serial port.

        *

        * Note that we would also want to call GCSuppressFinalize (regardless of the

        * value of disposing) if this class were finalizable (i.e. if it had a finalizer).

        * In this case, there are no direct refs to an unmanaged resource, so custom finalization

        * code isnt necessary.

        */

        private void Dispose(bool disposing)

        {

            if (mUSB == true)

            {

                // USB

                if (!disposed && disposing && m_hPort != 0)

                {

                    fContinue = false;

                    FT_Close(m_hPort);

                    m_hPort = 0;

                    // Keep us from calling resetting or closing multiple times

                    disposed = true;

                    fLog.WriteLine("--> Modulo de comunicacao finalizado: " + DateTime.Now.ToString());

                    fLog.Close();

                }

            }

            else

            {

                // COM

                if (!disposed && disposing && mSerialPort != null && mSerialPort.IsOpen)

                {

                    mSerialPort.Close();

                    // Keep us from calling resetting or closing multiple times

                    disposed = true;

                    fLog.WriteLine("--> Modulo de comunicacao finalizado: " + DateTime.Now.ToString());

                    fLog.Close();

                }

            }

        }

        #endregion

        #region >>> Propriedades <<<

        public bool TimedOut

        {

            get

            { return _timedOut; }

            set

            { _timedOut = value; }

        }

        public bool OnGetData

        {

            get

            { return mOnGetData; }

            set

            { mOnGetData = value; }

        }

        public bool SerialPortStop

        {

            get

            { return mSerialPortStop; }

            set

            { mSerialPortStop = value; }

        }

        public int[] FluxometrosConectados

        {

            get

            { return mFluxSerial; }

            set

            { mFluxSerial = value; }

        }

        #endregion

        #region >>> Método - BufferizarByteSemEscapear <<<

        /// <summary>

        /// Método utilizado para colocar um byte no buffer.

        /// </summary>

        private void BufferizarByteSemEscapear(byte b)

        {

            try

            {

                fBufferOut[fTamBufferOut] = b;

                fTamBufferOut++;

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: BufferizarByteSemEscapear(byte b) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - BufferizarByte <<<

        /// <summary>

        /// Método utilizado para colocar um byte no buffer. Escapeia se for necessário

        /// </summary>

        private void BufferizarByte(byte b)

        {

            try

            {

                if (b >= 0xFF)

                {

                    fBufferOut[fTamBufferOut] = 0xFF;

                    fTamBufferOut++;

                    b = (byte)(b - 0x20);

                }

                fBufferOut[fTamBufferOut] = b;

                fTamBufferOut++;

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: BufferizarByte(byte b) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - NovoPacote <<<

        /// <summary>

        /// Método utilizado para colocar um long no buffer. Escapeia se for necessário

        /// </summary>

        private void NovoPacote(byte Comando)

        {

            try

            {

                BufferizarByteSemEscapear(codStart);

                BufferizarByte(Comando);

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: NovoPacote(byte Comando) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - FecharEnviarPacote <<<

        /// <summary>

        /// Método utilizado para Fecha o pacote e mandar

        /// </summary>

        private void FecharEnviarPacote()

        {

            try

            {

                BufferizarByteSemEscapear(codStop);

                Send(fBufferOut);

                fBufferOut = new byte[MAX_BUFFER_SIZE];

                fTamBufferOut = 0;

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: FecharEnviarPacote() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }



        #endregion

        #region >>> Método - ccbConfigurarCentral <<<

        /// <summary>

        /// Método utilizado para configurar a central

        /// </summary>

        public unsafe Boolean ccbConfigurarCentral()

        {

            try

            {

                if (mUSB == true)

                {

                    // USB

                    UInt32 dwOpenFlag;

                    ListUnopenDevices();

                    FT_STATUS ftStatus = clsControladorRS232.FT_STATUS.FT_OTHER_ERROR;

                    if (m_hPort == 0)

                    {

                        dwOpenFlag = dwListDescFlags & ~FT_LIST_BY_INDEX;

                        dwOpenFlag = dwListDescFlags & ~FT_LIST_ALL;

                        System.Text.ASCIIEncoding enc = new System.Text.ASCIIEncoding();

                        byte[] sDevName = enc.GetBytes(mDeviceName);

                        fixed (byte* pBuf = sDevName)

                        {

                            ftStatus = FT_OpenEx(pBuf, dwOpenFlag, ref m_hPort);

                        }

                    }

                    if (ftStatus == clsControladorRS232.FT_STATUS.FT_OK)

                    {

                        // Set up the port

                        FT_SetBaudRate(m_hPort, (uint)mBaudRate);

                        FT_Purge(m_hPort, FT_PURGE_RX | FT_PURGE_TX);

                        FT_SetTimeouts(m_hPort, 10, 10);

                        // Start up the read and write thread

                        fLog.WriteLine("--> Modulo de comunicacao configurado USB: " + mDeviceName.Trim() + " " + DateTime.Now.ToString());

                        ccbGetInfo();

                        ccbGetDateTime();

                        if (bFlux40 == false)

                        {

                            ccbGetFluxCount();

                            ccbGetData();

                        }

                        fContinue = true;

                        pThreadRead = new Thread(new ThreadStart(OnDataReceived));

                        pThreadRead.Start();

                        return false;

                    }

                    else

                    {

                        throw new Exception("Failed To Open Port" + Convert.ToString(ftStatus));

                    }

                }

                else

                {

                    // COM

                    mSerialPort = new SerialPort();

                    mSerialPort.PortName = mSerialPortCom;

                    mSerialPort.BaudRate = mBaudRate;

                    mSerialPort.DataBits = 8;

                    mSerialPort.Parity = Parity.None;

                    mSerialPort.StopBits = StopBits.One;

                    mSerialPort.ParityReplace = 0x00;

                    mSerialPort.ReadTimeout = 100;

                    mSerialPort.WriteTimeout = 100;

                    mSerialPort.Open();

                    ccbGetInfo();

                    ccbGetDateTime();

                    ccbGetFluxCount();

                    pThreadRead = new Thread(new ThreadStart(OnDataReceived));

                    pThreadRead.Start();

                    fLog.WriteLine("--> Modulo de comunicacao configurado na porta: " + mSerialPortCom + " " + DateTime.Now.ToString());

                    return false;

                }

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbConfigurarCentral " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ListUnopenDevices <<<

        private unsafe bool ListUnopenDevices()

        {

            bool bRet = false;

            try

            {

                FT_STATUS ftStatus = clsControladorRS232.FT_STATUS.FT_OTHER_ERROR;

                UInt32 numDevs;

                int i;

                byte[] sDevName = new byte[64];

                void* p1;

                p1 = (void*)&numDevs;

                ftStatus = FT_ListDevices(p1, null, FT_LIST_NUMBER_ONLY);

                if (ftStatus == clsControladorRS232.FT_STATUS.FT_OK)

                {

                    for (i = 0; i < numDevs; i++)

                    {

                        fixed (byte* pBuf = sDevName)

                        {

                            ftStatus = FT_ListDevices((UInt32)i, pBuf, dwListDescFlags);

                            if (ftStatus == clsControladorRS232.FT_STATUS.FT_OK)

                            {

                                string str;

                                System.Text.ASCIIEncoding enc = new System.Text.ASCIIEncoding();

                                str = enc.GetString(sDevName, 0, sDevName.Length);

                                mDeviceName = str;

                                bRet = true;

                            }

                            else

                            {

                                throw new Exception("Error list devices" + Convert.ToString(ftStatus));

                            }

                        }

                    }

                }

            }

            catch (Exception ex)

            {

                bRet = false;

                fLog.WriteLine("--> Erro: ccbConfigurarCentralFTD2XX " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

            return bRet;

        }



        #endregion

        #region >>> Método - ccbGetInfo <<<

        /// <summary>

        /// Método utilizado para obter numero de serie da central

        /// </summary>

        public void ccbGetInfo()

        {

            try

            {

                NovoPacote(cmdGet + cmdGetInfo);

                FecharEnviarPacote();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbGetInfo() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ccbGetDateTime <<<

        /// <summary>

        /// Método utilizado para obter a data e hora da central

        /// </summary>

        public void ccbGetDateTime()

        {

            try

            {

                NovoPacote(cmdGet + cmdGetDateTime);

                FecharEnviarPacote();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbGetDateTime() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ccbGetFluxCount <<<

        /// <summary>

        /// Método utilizado para obter numero de fluxometros conectados

        /// </summary>

        public void ccbGetFluxCount()

        {

            try

            {

                NovoPacote(cmdGet + cmdGetFluxCount);

                FecharEnviarPacote();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbGetFluxCount() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ccbGetFluxSerial <<<

        /// <summary>

        /// Método utilizado para obter numero de serie dos fluxometros conectados

        /// </summary>

        public void ccbGetFluxCount(byte fIndiceFluxometro)

        {

            try

            {

                NovoPacote(cmdGet + cmdGetFluxSerial);

                BufferizarByte(fIndiceFluxometro);

                FecharEnviarPacote();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbGetFluxCount(byte fIndiceFluxometro) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ccbGetData <<<

        /// <summary>

        /// Método utilizado para iniciar envio de dados da central

        /// </summary>

        public void ccbGetData()

        {

            try

            {

                NovoPacote(cmdGet + cmdGetData);

                FecharEnviarPacote();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbGetData() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ccbResetCentral <<<

        /// <summary>

        /// Método utilizado para reset central

        /// </summary>

        public void ccbResetCentral()

        {

            try

            {

                NovoPacote(cmdGet + cmdResetCentral);

                FecharEnviarPacote();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ccbResetCentral() " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - OnDataReceived <<<

        /// <summary>

        /// Método utilizado para enviar dados para a porta serial

        /// </summary>

        private unsafe void OnDataReceived()

        {

            try

            {

                if (mUSB == true)

                {

                    // USB

                    byte[] cBuf = new Byte[MAX_DATA_LENGTH];

                    UInt32 dwRet = 0;

                    FT_STATUS ftStatus = clsControladorRS232.FT_STATUS.FT_OTHER_ERROR;

                    fBufferRead = new byte[MAX_DATA_LENGTH];

                    fTamBufferRead = 0;

                    while (fContinue == true)

                    {

                        if (m_hPort == 0)

                        {

                            return;

                        }

                        cBuf = new Byte[MAX_DATA_LENGTH];

                        fixed (byte* pBuf = cBuf)

                        {

                            ftStatus = FT_Read(m_hPort, pBuf, (uint)cBuf.Length, ref dwRet);

                        }

                        if (dwRet > 0 && ftStatus == clsControladorRS232.FT_STATUS.FT_OK)

                        {

                            fBufferRead = new byte[dwRet];

                            for (uint rx = 0; rx <= dwRet - 1; rx++)

                            {

                                fBufferRead[rx] = cBuf[rx];

                            }

                            if (ServiceDebug == "Y")

                            {

                                fLog.WriteLine("--> OnDataReceived: " + AtoX(fBufferRead) + " " + DateTime.Now.ToLongTimeString());

                            }

                            ProcessarDadosRecebidos(fBufferRead);

                        }

                    }

                }

                else

                {

                    // COM

                    byte[] cBuf = new Byte[MAX_DATA_LENGTH];

                    Int32 dwRet = 0;

                    fBufferRead = new byte[MAX_DATA_LENGTH];

                    fTamBufferRead = 0;

                    while (true)

                    {

                        try

                        {

                            cBuf = new Byte[MAX_DATA_LENGTH];

                            dwRet = mSerialPort.Read(cBuf, 0, MAX_DATA_LENGTH);

                        }

                        catch (TimeoutException)

                        {

                            if (ServiceDebug == "Y")

                            {

                                fLog.WriteLine("--> TimeOut: " + DateTime.Now.ToLongTimeString());

                            }

                        }

                        if (dwRet > 0)

                        {

                            fBufferRead = new byte[dwRet];

                            for (uint rx = 0; rx <= dwRet - 1; rx++)

                            {

                                fBufferRead[rx] = cBuf[rx];

                            }

                            if (ServiceDebug == "Y")

                            {

                                fLog.WriteLine("--> OnDataReceived: " + AtoX(fBufferRead) + " " + DateTime.Now.ToLongTimeString());

                            }

                            ProcessarDadosRecebidos(fBufferRead);

                        }

                    }

                }

            }

            catch (IOException)

            {

                // abort the thread

                System.Threading.Thread.CurrentThread.Abort();

            }

            catch (ObjectDisposedException)

            {

                if (pThreadRead != null)

                {

                    pThreadRead = null;

                }

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: OnDataReceived: " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ProcessarDadosRecebidos <<<

        /// <summary>

        /// Método utilizado para processar pacote de dados recebidos da central

        /// </summary>

        private void ProcessarDadosRecebidos(byte[] data)

        {

            try

            {

                foreach (byte b in data)

                {

                    if (b == 0xFA)

                    {

                        fStart = true;

                        fPosBufferIn = 0;

                        fBufferIn = new byte[MAX_BUFFER_SIZE];

                        continue;

                    }

                    if (b == 0xFB)

                    {

                        if (fStart)

                        {

                            if (ServiceDebug == "Y")

                            {

                                fLog.WriteLine("--> fBufferIn: " + AtoX(fBufferIn) + " " + DateTime.Now.ToLongTimeString());

                            }

                            if (fPosBufferIn >= 0)

                            {

                                ProcessarComando(fBufferIn[0]);

                            }

                            fPosBufferIn = 0;

                            fBufferIn = new byte[MAX_BUFFER_SIZE];

                        }

                        fStart = false;

                        continue;

                    }

                    if (b >= 0xFF)

                    {

                        fEscapeado = true;

                        continue;

                    }

                    if (fPosBufferIn > MAX_BUFFER_SIZE - 1)

                    {

                        continue;

                    }

                    if (fEscapeado == true)

                    {

                        fBufferIn[fPosBufferIn] = (byte)(b + 0x20);

                        if (fPosBufferIn <= MAX_BUFFER_SIZE - 1)

                        {

                            fPosBufferIn++;

                        }

                        fEscapeado = false;

                    }

                    else

                    {

                        fBufferIn[fPosBufferIn] = b;

                        if (fPosBufferIn <= MAX_BUFFER_SIZE - 1)

                        {

                            fPosBufferIn++;

                        }

                    }

                }

                return;

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ProcessarDadosRecebidos(byte[] data) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - ProcessarComando <<<

        /// <summary>

        /// Método utilizado para processar comando recebido da central

        /// </summary>

        private void ProcessarComando(byte comando)

        {

            try

            {

                mEventoDataInicial = DateTime.Now;

                mEventoDataFinal = DateTime.Now;

                switch (comando)

                {

                    case 0x81: // Info

                        mEvento = true;

                        mCentralSerial = 0;

                        mCentralVersao = 0;

                        vlr = ((long)fBufferIn[1] << 16) +

                        ((long)fBufferIn[2] << 8) +

                        ((long)fBufferIn[3]);

                        mCentralSerial = Convert.ToInt32(vlr);

                        string v1 = "0";

                        string v2 = "0";

                        string v3 = "0";



                        try

                        {

                            v1 = (Convert.ToInt32(Convert.ToString(fBufferIn[4], 16)) - 30).ToString();

                            v2 = (Convert.ToInt32(Convert.ToString(fBufferIn[5], 16)) - 30).ToString();

                            v3 = (Convert.ToInt32(Convert.ToString(fBufferIn[6], 16)) - 30).ToString();

                        }

                        catch

                        {

                            v1 = "0";

                            v2 = "0";

                            v3 = "0";

                        }

                        mCentralVersao = Convert.ToInt32(v1 + v2 + v3);

                        if (mCentralVersao == 353)

                        {

                            bFlux40 = true;

                        }

                        else

                        {

                            bFlux40 = false;

                        }

                        if (mCentralVersao < 440)

                        {

                            Firmware = 400;

                        }

                        else

                        {

                            Firmware = 440;

                        }

                        mEventoTipo = 768;

                        mEventoID = mCentralSerial;

                        mEventoValor = mCentralVersao;

                        mEventoModulo = mCentralSerial;

                        break;

                    case 0x82: // PowerInfo

                        mEvento = true;

                        if (fBufferIn[1] == 0)

                        {

                            mEventoTipo = 769;

                        }

                        else

                        {

                            mEventoTipo = 770;

                        }

                        vlr = (long)(fBufferIn[2] << 8) +

                        (long)(fBufferIn[3]);

                        mEventoValor = Convert.ToInt32(vlr);

                        break;

                    case 0x83: // DateTime

                        mEvento = false;

                        dti = ((long)fBufferIn[1] << 24) +

                        ((long)fBufferIn[2] << 16) +

                        ((long)fBufferIn[3] << 8) +

                        ((long)fBufferIn[4]);

                        mEventoDTI = DateTime.Now;

                        mEventoDTA = mEventoDTI.AddSeconds(dti / 100);

                        TimeSpan mDif = mEventoDTA.Subtract(mEventoDTI);

                        mEventoDTI = mEventoDTI.Subtract(mDif);

                        mEventoDTF = mEventoDTI.Add(mDif);

                        mEventoDTA = mEventoDTF;

                        mEventoDTS = mEventoDTF;

                        mCCBDti = dti;

                        RTCAnterior = dti;

                        RTCTempo = 0;

                        break;

                    case 0x87: // FluxCount

                        mEvento = true;

                        mFluxCount = fBufferIn[1];

                        if (mFluxCount > 0)

                        {

                            bFluxometrosConectados = true;

                        }

                        for (int index = 0; index < mFluxCount; index++)

                        {

                            NovoPacote(cmdGet + cmdGetFluxSerial);

                            BufferizarByte((byte)index);

                            FecharEnviarPacote();

                        }

                        break;

                    case 0x88: // FluxSerial

                        mEvento = true;

                        if (fBufferIn[2] == 0)

                        {

                            mEventoTipo = 514;

                        }

                        else

                        {

                            mEventoTipo = 513;

                        }

                        mFluxAtivo[fBufferIn[1]] = fBufferIn[2];

                        vlr = ((long)fBufferIn[3] << 16) +

                        ((long)fBufferIn[4] << 8) +

                        ((long)fBufferIn[5]);

                        mEventoID = Convert.ToInt32(vlr);

                        mFluxSerial[fBufferIn[1]] = Convert.ToInt32(vlr);

                        DataSet ds = new DataSet();

                        ds = ControladorProduto.ObterFluxometroID(mEventoID);

                        if (ds.Tables[0].Rows.Count > 0)

                        {

                            // SetThresholdPulse

                            NovoPacote(cmdGet + cmdSetThresholdPulse);

                            long limiar = Convert.ToInt32(ds.Tables[0].Rows[0]["Velocidade"]);

                            byte b1 = fBufferIn[1];

                            byte b2 = (byte)(limiar >> 8);

                            byte b3 = (byte)(limiar);

                            BufferizarByte(b1);

                            BufferizarByte(b2);

                            BufferizarByte(b3);

                            FecharEnviarPacote();

                        }

                        break;

                    case 0xA0: // FluxConnectStatus

                        mEvento = true;

                        if (bFluxometrosConectados == true)

                        {

                            mEvento = true;

                            if (fBufferIn[2] == 0)

                            {

                                mEventoTipo = 514;

                            }

                            else

                            {

                                mEventoTipo = 513;

                            }

                        }

                        vlr = ((long)fBufferIn[3] << 16) +

                        ((long)fBufferIn[4] << 8) +

                        ((long)fBufferIn[5]);

                        mEventoID = Convert.ToInt32(vlr);

                        NovoPacote(cmdGet + cmdAckFluxConnectStatus);

                        BufferizarByte(fBufferIn[1]);

                        BufferizarByte(fBufferIn[3]);

                        BufferizarByte(fBufferIn[4]);

                        BufferizarByte(fBufferIn[5]);

                        FecharEnviarPacote();

                        break;

                    case 0xA1: // FluxBadPulses

                        mEvento = true;

                        mEventoTipo = 257;

                        if (Firmware == 400)

                        {

                            // Versão Firmware 4.0.0

                            mEventoIndiceFlux = fBufferIn[1];

                            vlr = ((long)fBufferIn[2] << 16) +

                            ((long)fBufferIn[3] << 8) +

                            ((long)fBufferIn[4]);

                            mEventoID = Convert.ToInt32(vlr);

                            dti = ((long)fBufferIn[5] << 24) +

                            ((long)fBufferIn[6] << 16) +

                            ((long)fBufferIn[7] << 8) +

                            ((long)fBufferIn[8]);

                            dtf = ((long)fBufferIn[9] << 24) +

                            ((long)fBufferIn[10] << 16) +

                            ((long)fBufferIn[11] << 8) +

                            ((long)fBufferIn[12]);

                            vlr = ((long)fBufferIn[13] << 24) +

                            ((long)fBufferIn[14] << 16) +

                            ((long)fBufferIn[15] << 8) +

                            ((long)fBufferIn[16]);

                            vlrcrc = (long)(fBufferIn[17] << 8) +

                            (long)(fBufferIn[18]);

                            calculatedCRC = CRCGenerator.GenerateCRC(fBufferIn, 0, 17, CRC_SEED);

                        }

                        else

                        {

                            // Versão Firmware 4.4.0

                            mEventoIndiceFlux = fBufferIn[1];

                            mEventoStatusFlux = fBufferIn[2];

                            if (mEventoStatusFlux == 0)

                            {

                                mEventoMensagem = "FD";

                            }

                            else

                            {

                                mEventoMensagem = "FC";

                            }

                            vlr = ((long)fBufferIn[3] << 16) +

                            ((long)fBufferIn[4] << 8) +

                            ((long)fBufferIn[5]);

                            mEventoID = Convert.ToInt32(vlr);

                            dti = ((long)fBufferIn[6] << 24) +

                            ((long)fBufferIn[7] << 16) +

                            ((long)fBufferIn[8] << 8) +

                            ((long)fBufferIn[9]);

                            dtf = ((long)fBufferIn[10] << 24) +

                            ((long)fBufferIn[11] << 16) +

                            ((long)fBufferIn[12] << 8) +

                            ((long)fBufferIn[13]);

                            vlr = ((long)fBufferIn[14] << 24) +

                            ((long)fBufferIn[15] << 16) +

                            ((long)fBufferIn[16] << 8) +

                            ((long)fBufferIn[17]);

                            vlrcrc = (long)(fBufferIn[18] << 8) +

                            (long)(fBufferIn[19]);

                            calculatedCRC = CRCGenerator.GenerateCRC(fBufferIn, 0, 18, CRC_SEED);

                        }

                        // Verifica CRC

                        if (calculatedCRC == vlrcrc)

                        {

                            if (mCCBFluxometroID != mEventoID || mCCBDti != dti)

                            {

                                mCCBGravar = true;

                                mCCBFluxometroID = mEventoID;

                                mCCBDti = dti;

                            }

                            else

                            {

                                mCCBGravar = false;

                            }

                            RTCTempo = dti - RTCAnterior;

                            if (RTCTempo != 0)

                            {

                                mEventoDTI = mEventoDTA.AddSeconds(RTCTempo / 100);

                                if (dtf / 100 != 0)

                                {

                                    mEventoDTF = mEventoDTI.AddSeconds(dtf / 100);

                                }

                                else

                                {

                                    mEventoDTF = mEventoDTI;

                                }

                            }

                            RTCAnterior = dti;

                            mEventoDataInicial = mEventoDTI;

                            mEventoDataFinal = mEventoDTF;

                            mEventoDTA = mEventoDTI;

                            mEventoTempo = (double)dtf / 100;

                            mEventoValor = Convert.ToInt32(vlr);

                            NovoPacote(cmdGet + cmdAckFluxBadPulses);

                            BufferizarByte(fBufferIn[1]);

                            FecharEnviarPacote();

                        }

                        else

                        {

                            mCCBGravar = false;

                            mEventoTipo = 301;

                            fLog.WriteLine("--> ERRO CRC " + DateTime.Now.ToString());

                        }

                        break;

                    case 0xA2: // FluxGoodPulses

                        mEvento = true;

                        mEventoTipo = 258;

                        if (Firmware == 400)

                        {

                            // Versão Firmware 4.0.0

                            mEventoIndiceFlux = fBufferIn[1];

                            vlr = ((long)fBufferIn[2] << 16) +

                            ((long)fBufferIn[3] << 8) +

                            ((long)fBufferIn[4]);

                            mEventoID = Convert.ToInt32(vlr);

                            dti = ((long)fBufferIn[5] << 24) +

                            ((long)fBufferIn[6] << 16) +

                            ((long)fBufferIn[7] << 8) +

                            ((long)fBufferIn[8]);

                            dtf = ((long)fBufferIn[9] << 24) +

                            ((long)fBufferIn[10] << 16) +

                            ((long)fBufferIn[11] << 8) +

                            ((long)fBufferIn[12]);

                            vlr = ((long)fBufferIn[13] << 24) +

                            ((long)fBufferIn[14] << 16) +

                            ((long)fBufferIn[15] << 8) +

                            ((long)fBufferIn[16]);

                            vlrcrc = (long)(fBufferIn[17] << 8) +

                            (long)(fBufferIn[18]);

                            calculatedCRC = CRCGenerator.GenerateCRC(fBufferIn, 0, 17, CRC_SEED);

                        }

                        else

                        {

                            // Versão Firmware 4.4.0

                            mEventoIndiceFlux = fBufferIn[1];

                            mEventoStatusFlux = fBufferIn[2];

                            if (mEventoStatusFlux == 0)

                            {

                                mEventoMensagem = "FD";

                            }

                            else

                            {

                                mEventoMensagem = "FC";

                            }

                            vlr = ((long)fBufferIn[3] << 16) +

                            ((long)fBufferIn[4] << 8) +

                            ((long)fBufferIn[5]);

                            mEventoID = Convert.ToInt32(vlr);

                            dti = ((long)fBufferIn[6] << 24) +

                            ((long)fBufferIn[7] << 16) +

                            ((long)fBufferIn[8] << 8) +

                            ((long)fBufferIn[9]);

                            dtf = ((long)fBufferIn[10] << 24) +

                            ((long)fBufferIn[11] << 16) +

                            ((long)fBufferIn[12] << 8) +

                            ((long)fBufferIn[13]);

                            vlr = ((long)fBufferIn[14] << 24) +

                            ((long)fBufferIn[15] << 16) +

                            ((long)fBufferIn[16] << 8) +

                            ((long)fBufferIn[17]);

                            vlrcrc = (long)(fBufferIn[18] << 8) +

                            (long)(fBufferIn[19]);

                            calculatedCRC = CRCGenerator.GenerateCRC(fBufferIn, 0, 18, CRC_SEED);

                        }

                        // Verifica CRC

                        if (calculatedCRC == vlrcrc)

                        {

                            if (mCCBFluxometroID != mEventoID || mCCBDti != dti)

                            {

                                mCCBGravar = true;

                                mCCBFluxometroID = mEventoID;

                                mCCBDti = dti;

                            }

                            else

                            {

                                mCCBGravar = false;

                            }

                            RTCTempo = dti - RTCAnterior;

                            if (RTCTempo != 0)

                            {

                                mEventoDTI = mEventoDTA.AddSeconds(RTCTempo / 100);

                                if (dtf / 100 != 0)

                                {

                                    mEventoDTF = mEventoDTI.AddSeconds(dtf / 100);

                                }

                                else

                                {

                                    mEventoDTF = mEventoDTI;

                                }

                            }

                            RTCAnterior = dti;

                            mEventoDataInicial = mEventoDTI;

                            mEventoDataFinal = mEventoDTF;

                            mEventoDTA = mEventoDTI;

                            mEventoTempo = (double)dtf / 100;

                            mEventoValor = Convert.ToInt32(vlr);

                            NovoPacote(cmdGet + cmdAckFluxGoodPulses);

                            BufferizarByte(fBufferIn[1]);

                            FecharEnviarPacote();

                        }

                        else

                        {

                            mCCBGravar = false;

                            mEventoTipo = 301;

                            fLog.WriteLine("--> ERRO CRC " + DateTime.Now.ToString());

                        }

                        break;

                    case 0xA3: // FluxBadPulsesLog

                        mEvento = true;

                        mEventoTipo = 259;

                        if (Firmware == 400)

                        {

                            // Versão Firmware 4.0.0

                            mEventoIndiceFlux = fBufferIn[1];

                            vlr = ((long)fBufferIn[2] << 16) +

                            ((long)fBufferIn[3] << 8) +

                            ((long)fBufferIn[4]);

                            mEventoID = Convert.ToInt32(vlr);

                            dti = ((long)fBufferIn[5] << 24) +

                            ((long)fBufferIn[6] << 16) +

                            ((long)fBufferIn[7] << 8) +

                            ((long)fBufferIn[8]);

                            dtf = ((long)fBufferIn[9] << 24) +

                            ((long)fBufferIn[10] << 16) +

                            ((long)fBufferIn[11] << 8) +

                            ((long)fBufferIn[12]);

                            vlr = ((long)fBufferIn[13] << 24) +

                            ((long)fBufferIn[14] << 16) +

                            ((long)fBufferIn[15] << 8) +

                            ((long)fBufferIn[16]);

                            vlrcrc = (long)(fBufferIn[17] << 8) +

                            (long)(fBufferIn[18]);

                            calculatedCRC = CRCGenerator.GenerateCRC(fBufferIn, 0, 17, CRC_SEED);

                        }

                        else

                        {

                            // Versão Firmware 4.4.0

                            mEventoIndiceFlux = fBufferIn[1];

                            mEventoStatusFlux = fBufferIn[2];

                            if (mEventoStatusFlux == 0)

                            {

                                mEventoMensagem = "FD";

                            }

                            else

                            {

                                mEventoMensagem = "FC";

                            }

                            vlr = ((long)fBufferIn[3] << 16) +

                            ((long)fBufferIn[4] << 8) +

                            ((long)fBufferIn[5]);

                            mEventoID = Convert.ToInt32(vlr);

                            dti = ((long)fBufferIn[6] << 24) +

                            ((long)fBufferIn[7] << 16) +

                            ((long)fBufferIn[8] << 8) +

                            ((long)fBufferIn[9]);

                            dtf = ((long)fBufferIn[10] << 24) +

                            ((long)fBufferIn[11] << 16) +

                            ((long)fBufferIn[12] << 8) +

                            ((long)fBufferIn[13]);

                            vlr = ((long)fBufferIn[14] << 24) +

                            ((long)fBufferIn[15] << 16) +

                            ((long)fBufferIn[16] << 8) +

                            ((long)fBufferIn[17]);

                            vlrcrc = (long)(fBufferIn[18] << 8) +

                            (long)(fBufferIn[19]);

                            calculatedCRC = CRCGenerator.GenerateCRC(fBufferIn, 0, 18, CRC_SEED);

                        }

                        // Verifica CRC

                        if (calculatedCRC == vlrcrc)

                        {

                            if (mCCBFluxometroID != mEventoID || mCCBDti != dti)

                            {

                                mCCBGravar = true;

                                mCCBFluxometroID = mEventoID;

                                mCCBDti = dti;

                            }

                            else

                            {

                                mCCBGravar = false;

                            }

                            RTCTempo = dti - RTCAnterior;

                            if (RTCTempo != 0)

                            {

                                mEventoDTI = mEventoDTA.AddSeconds(RTCTempo / 100);

                                if (dtf / 100 != 0)

                                {

                                    mEventoDTF = mEventoDTI.AddSeconds(dtf / 100);

                                }

                                else

                                {

                                    mEventoDTF = mEventoDTI;

                                }

                            }

                            RTCAnterior = dti;

                            mEventoDataInicial = mEventoDTI;

                            mEventoDataFinal = mEventoDTF;

                            mEventoDTA = mEventoDTI;

                            mEventoTempo = (double)dtf / 100;

                            mEventoValor = Convert.ToInt32(vlr);

                            NovoPacote(cmdGet + cmdAckFluxBadPulsesLog);

                            BufferizarByte(fBufferIn[1]);

                            FecharEnviarPacote();

                        }

                        else

                        {

                            mCCBGravar = false;

                            mEventoTipo = 301;

                            fLog.WriteLine("--> ERRO CRC " + DateTime.Now.ToString());

                        }

                        break;

                    default:

                        mEvento = false;

                        mEventoTipo = comando;

                        break;

                }

                if (mEvento == true)

                {

                    mEventoNumero++;

                    if (mEventoTipo == 257 || mEventoTipo == 258 || mEventoTipo == 259)

                    {

                        if (bFlux40 == true)

                        {

                            mEventoID++;

                            mEventoValor++;

                        }

                        if (mCCBGravar == true)

                        {

                            if (ServiceDebug == "Y")

                            {

                                fLog.WriteLine("--> OnRS232ReceviedHandler: " +

                                mEventoModulo.ToString() + " " +

                                mEventoNumero.ToString() + " " +

                                mEventoTipo.ToString() + " " +

                                mEventoID.ToString() + " " +

                                mEventoDataInicial.ToString() + " " +

                                mEventoDataFinal.ToString() + " " +

                                mEventoValor.ToString() + " " +

                                mEventoMensagem.Trim() + " " +

                                mEventoTempo.ToString("###.##0") + " " +

                                DateTime.Now.ToLongTimeString() + " " +

                                dti.ToString() + " " +

                                dtf.ToString());

                            }

                            OnRS232ReceviedHandler(this,

                            mEventoModulo,

                            mEventoNumero,

                            mEventoTipo,

                            mEventoID,

                            mEventoDataInicial,

                            mEventoDataFinal,

                            mEventoValor,

                            mEventoMensagem,

                            mEventoTempo);

                        }

                    }

                    else

                    {

                        if (ServiceDebug == "Y")

                        {

                            fLog.WriteLine("--> OnRS232ReceviedHandler: " +

                            mEventoModulo.ToString() + " " +

                            mEventoNumero.ToString() + " " +

                            mEventoTipo.ToString() + " " +

                            mEventoID.ToString() + " " +

                            mEventoDataInicial.ToString() + " " +

                            mEventoDataFinal.ToString() + " " +

                            mEventoValor.ToString() + " " +

                            mEventoMensagem.Trim() + " " +

                            DateTime.Now.ToLongTimeString());

                        }

                        OnRS232ReceviedHandler(this,

                        mEventoModulo,

                        mEventoNumero,

                        mEventoTipo,

                        mEventoID,

                        mEventoDataInicial,

                        mEventoDataFinal,

                        mEventoValor,

                        mEventoMensagem,

                        mEventoTempo);

                    }

                }

                mEvento = false;

                mEventoTipo = 0;

                mEventoID = 0;

                mEventoDataInicial = DateTime.Now;

                mEventoDataFinal = DateTime.Now;

                mEventoValor = 0;

                mEventoMensagem = string.Empty;

                mEventoTempo = 0;

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: ProcessarComando(byte comando) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - Send <<<

        /// <summary>

        /// Método utilizado para enviar dados para a FTD2XX

        /// </summary>

        private unsafe void Send(byte[] data)

        {

            try

            {

                // USB

                if (mUSB == true)

                {

                    UInt32 dwRet = 0;

                    FT_STATUS ftStatus = clsControladorRS232.FT_STATUS.FT_OTHER_ERROR;

                    fixed (byte* pBuf = data)

                    {

                        ftStatus = FT_Write(m_hPort, pBuf, (uint)(data.Length), ref dwRet);

                    }

                    if (ftStatus != clsControladorRS232.FT_STATUS.FT_OK)

                    {

                        throw new Exception("Failed To Write " + Convert.ToString(ftStatus));

                    }

                    if (ServiceDebug == "Y")

                    {

                        fLog.WriteLine("--> Send: " + AtoX(data) + " " + DateTime.Now.ToLongTimeString());

                    }

                }

                else

                {

                    // COM

                    if (null == data || data.Length > MAX_DATA_LENGTH)

                    {

                        fLog.WriteLine("--> Erro: bad data sent to Send -(null == data || data.Length > MAX_DATA_LENGTH) " + " " + DateTime.Now.ToLongTimeString());

                        throw new Exception("bad data sent to Send");

                    }



                    mSerialPort.Write(data, 0, data.Length);

                    if (ServiceDebug == "Y")

                    {

                        fLog.WriteLine("--> Send: " + AtoX(data) + " " + DateTime.Now.ToLongTimeString());

                    }

                }

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: Send(byte[] data) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

            }

            finally

            {

                fBufferOut = new byte[MAX_BUFFER_SIZE];

                fTamBufferOut = 0;

            }

        }

        #endregion

        #region >>> Método - AtoX <<<

        /// <summary>

        /// Converts an ASCII string to hex formatted lines.

        /// </summary>

        private string AtoX(byte[] data)

        {

            try

            {

                StringBuilder sb = new StringBuilder();

                for (int i = 0; i <= data.Length - 1; i++)

                {

                    sb.Append(Convert.ToString(data[i], 16) + " ");

                }

                return sb.ToString();

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: AtoX(byte[] data) " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

        }

        #endregion

        #region >>> Método - PesquisarDiretorioChoppControl <<<

        private string PesquisarDiretorioChoppControl()

        {

            string strChoppControlPath = string.Empty;

            try

            {

                string[] drives = Directory.GetLogicalDrives();

                foreach (string strPath in drives)

                {

                    if (strPath != "A:\\" & strPath != "B:\\")

                    {

                        try

                        {

                            foreach (string strDir in Directory.GetDirectories(strPath))

                            {

                                DirectoryInfo di = new DirectoryInfo(strDir);

                                DirectoryInfo[] dirs = di.GetDirectories("*Chopp Control 2007*");

                                if (dirs.Length > 0)

                                {

                                    strChoppControlPath = di.FullName;

                                }

                            }

                        }

                        catch

                        {

                        }

                    }

                }

            }

            catch (Exception ex)

            {

                fLog.WriteLine("--> Erro: PesquisarDiretorioChoppControl " + ex.ToString() + " " + DateTime.Now.ToLongTimeString());

                throw new Exception(ex.ToString());

            }

            return strChoppControlPath;

        }

        #endregion

        #region >>> Classe - CRCGenerator <<<

        /// <summary>

        /// Classe utilizada para gerar CRC

        /// </summary>

        public static class CRCGenerator

        {

            //CRC lookup table to avoid bit-shifting loops.

            static ushort[] crcLookupTable = {

0x00000, 0x01189, 0x02312, 0x0329B, 0x04624, 0x057AD, 0x06536, 0x074BF, 0x08C48, 0x09DC1, 0x0AF5A, 0x0BED3, 0x0CA6C, 0x0DBE5, 0x0E97E, 0x0F8F7, 0x01081, 0x00108, 0x03393, 0x0221A, 0x056A5, 0x0472C, 0x075B7, 0x0643E, 0x09CC9, 0x08D40, 0x0BFDB, 0x0AE52, 0x0DAED, 0x0CB64, 0x0F9FF, 0x0E876, 0x02102, 0x0308B, 0x00210, 0x01399, 0x06726, 0x076AF, 0x04434, 0x055BD, 0x0AD4A, 0x0BCC3, 0x08E58, 0x09FD1, 0x0EB6E, 0x0FAE7, 0x0C87C, 0x0D9F5, 0x03183, 0x0200A, 0x01291, 0x00318, 0x077A7, 0x0662E, 0x054B5, 0x0453C, 0x0BDCB, 0x0AC42, 0x09ED9, 0x08F50, 0x0FBEF, 0x0EA66, 0x0D8FD, 0x0C974, 0x04204, 0x0538D, 0x06116, 0x0709F, 0x00420, 0x015A9, 0x02732, 0x036BB, 0x0CE4C, 0x0DFC5, 0x0ED5E, 0x0FCD7, 0x08868, 0x099E1, 0x0AB7A, 0x0BAF3, 0x05285, 0x0430C, 0x07197, 0x0601E, 0x014A1, 0x00528, 0x037B3, 0x0263A, 0x0DECD, 0x0CF44, 0x0FDDF, 0x0EC56, 0x098E9, 0x08960, 0x0BBFB, 0x0AA72, 0x06306, 0x0728F, 0x04014, 0x0519D, 0x02522, 0x034AB, 0x00630, 0x017B9, 0x0EF4E, 0x0FEC7, 0x0CC5C, 0x0DDD5, 0x0A96A, 0x0B8E3, 0x08A78, 0x09BF1, 0x07387, 0x0620E, 0x05095, 0x0411C, 0x035A3, 0x0242A, 0x016B1, 0x00738, 0x0FFCF, 0x0EE46, 0x0DCDD, 0x0CD54, 0x0B9EB, 0x0A862, 0x09AF9, 0x08B70, 0x08408, 0x09581, 0x0A71A, 0x0B693, 0x0C22C, 0x0D3A5, 0x0E13E, 0x0F0B7, 0x00840, 0x019C9, 0x02B52, 0x03ADB, 0x04E64, 0x05FED, 0x06D76, 0x07CFF, 0x09489, 0x08500, 0x0B79B, 0x0A612, 0x0D2AD, 0x0C324, 0x0F1BF, 0x0E036, 0x018C1, 0x00948, 0x03BD3, 0x02A5A, 0x05EE5, 0x04F6C, 0x07DF7, 0x06C7E, 0x0A50A, 0x0B483, 0x08618, 0x09791, 0x0E32E, 0x0F2A7, 0x0C03C, 0x0D1B5, 0x02942, 0x038CB, 0x00A50, 0x01BD9, 0x06F66, 0x07EEF, 0x04C74, 0x05DFD, 0x0B58B, 0x0A402, 0x09699, 0x08710, 0x0F3AF, 0x0E226, 0x0D0BD, 0x0C134, 0x039C3, 0x0284A, 0x01AD1, 0x00B58, 0x07FE7, 0x06E6E, 0x05CF5, 0x04D7C, 0x0C60C, 0x0D785, 0x0E51E, 0x0F497, 0x08028, 0x091A1, 0x0A33A, 0x0B2B3, 0x04A44, 0x05BCD, 0x06956, 0x078DF, 0x00C60, 0x01DE9, 0x02F72, 0x03EFB, 0x0D68D, 0x0C704, 0x0F59F, 0x0E416, 0x090A9, 0x08120, 0x0B3BB, 0x0A232, 0x05AC5, 0x04B4C, 0x079D7, 0x0685E, 0x01CE1, 0x00D68, 0x03FF3, 0x02E7A, 0x0E70E, 0x0F687, 0x0C41C, 0x0D595, 0x0A12A, 0x0B0A3, 0x08238, 0x093B1, 0x06B46, 0x07ACF, 0x04854, 0x059DD, 0x02D62, 0x03CEB, 0x00E70, 0x01FF9, 0x0F78F, 0x0E606, 0x0D49D, 0x0C514, 0x0B1AB, 0x0A022, 0x092B9, 0x08330, 0x07BC7, 0x06A4E, 0x058D5, 0x0495C, 0x03DE3, 0x02C6A, 0x01EF1, 0x00F78

};

            public static ushort GenerateCRC(byte[] data, int dataLength, ushort seed)

            {

                ushort newCrc;

                newCrc = seed;

                for (int i = 0; i < dataLength; i++)

                {

                    newCrc = (ushort)((newCrc >> 8) ^ crcLookupTable[(newCrc ^ data[i]) & 0xff]);

                }

                return ((ushort)~newCrc);

            }

            public static ushort GenerateCRC(byte[] data, int startIndex, int length, ushort seed)

            {

                ushort newCrc;

                newCrc = seed;

                for (int i = 0; i < length; i++)

                {

                    newCrc = (ushort)((newCrc >> 8) ^ crcLookupTable[(newCrc ^ data[(startIndex + i) % data.Length]) & 0xff]);

                }

                return ((ushort)~newCrc);

            }

        }

        #endregion

    }

}