
using System;
using System.IO.Ports;
using System.Threading;
using WindowsInput;

namespace LeitorDeCodigo
{


    class Program
    {

        static void Main(string[] args)
        {
            var mySerial = new SerialPort();

            try
            {
                mySerial = MySerialPort();

                while (true)
                {

                    Thread.Sleep(3000);

                    var texto = mySerial.ReadExisting();


                    if (!string.IsNullOrEmpty(texto))
                    {
                        InputSimulator.SimulateTextEntry(texto);
                        InputSimulator.SimulateKeyPress(VirtualKeyCode.RETURN);
                    }

                }

            }

            catch (Exception e)
            {
                Console.WriteLine($"{e.Message} \nerro ocorrido na porta {mySerial.PortName}");
            }

            finally
            {
                mySerial.Close();
            }

            Console.ReadKey();
        }


        public static SerialPort MySerialPort()

        {
            var serial = new SerialPort();
            var serialPortNames = SerialPort.GetPortNames();
            var args = Environment.GetCommandLineArgs();

            if (args.Length == 1)
            {
                for (int i = 0; i < serialPortNames.Length; i++)
                {
                    var portas = serialPortNames[i];
                    serial.PortName = serialPortNames[0];
                }
            }

            else
            {

                var opcaoBool = VerificaPortaExistente(args[1]);

                if (opcaoBool)
                {
                    serial.PortName = args[1];
                }
                else
                {
                    throw new Exception("Porta não encontrada!");
                }
            }

            serial.BaudRate = 19200;
            serial.DataBits = 8;
            serial.Parity = Parity.None;
            serial.StopBits = StopBits.One;

            if (!serial.IsOpen)
            {
                serial.Open();
            }

            return serial;
        }


        public static bool VerificaPortaExistente(string args)
        {
            var valor = false;

            var nomePorta = SerialPort.GetPortNames();

            for (int i = 0; i < nomePorta.Length; i++)
            {
                var portas = i.ToString();

                if (args == portas)
                {
                    valor = true;
                    break;
                }

            }

            return valor;
        }
    }
}
