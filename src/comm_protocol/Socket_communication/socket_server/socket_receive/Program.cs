using System;
using System.Collections;
using System.ComponentModel;
using System.Threading;
using System.Runtime.InteropServices;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Globalization;
using System.Threading.Tasks;
using System.Net.NetworkInformation;
using System.Text;

namespace socket_receive
{
    public class StateObject
    {
        public Socket workSocket = null;
        public const int BufferSize = 3000;
        public byte[] buffer = new byte[BufferSize];
    }
    class Program
    {
        private static ManualResetEvent allDoneLAN = new ManualResetEvent(false);

        static void Main()
        {
            Thread lanListen = new Thread(socketListener);
            lanListen.Name = "Socket Server";
            lanListen.Start();
            while (true) ;
        }

        private static void socketListener()
        {
            IPGlobalProperties computerProperties = IPGlobalProperties.GetIPGlobalProperties();
            NetworkInterface[] nics = NetworkInterface.GetAllNetworkInterfaces();
            IPHostEntry ipHostInfo = Dns.Resolve(Dns.GetHostName());
            IPEndPoint localEP = new IPEndPoint(ipHostInfo.AddressList[2], Convert.ToInt16("9999"));
            Console.WriteLine("IP : {0}", localEP.ToString());
            Socket listener = new Socket(localEP.Address.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

            try
            {
                listener.Bind(localEP);
                listener.Listen(10);
                Console.WriteLine("Socket server started..!");
                while (true)
                {
                    allDoneLAN.Reset();
                    listener.BeginAccept(new AsyncCallback(acceptSocketCallback), listener);
                    allDoneLAN.WaitOne();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error : Socket Error in socketListener().!\nDetails : " + ex.ToString());
            }
            Console.WriteLine("Closing the listener...");
        }
        public static void acceptSocketCallback(IAsyncResult ar)
        {
            try
            {

                Socket listener = (Socket)ar.AsyncState;
                Socket handler = listener.EndAccept(ar);
                allDoneLAN.Set();
                Console.WriteLine("Client connected...");
                StateObject state = new StateObject();
                state.workSocket = handler;
                handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0,
                    new AsyncCallback(readSocketCallback), state);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error : Socket Error in acceptSocketCallback().!\nDetails : " + ex.ToString());
            }
        }

        public static void readSocketCallback(IAsyncResult ar)
        {
            try
            {
                StateObject state = (StateObject)ar.AsyncState;
                Socket handler = state.workSocket;
                int read = handler.EndReceive(ar);
                if (read != 0)  // detect socket close
                {
                    char[] receivedData = new char[read];
                    Array.Copy(state.buffer, receivedData, receivedData.Length);
                    string data_soc = new string(receivedData);
                    Console.WriteLine("Received data : \"{0}\" ({1} bytes)", data_soc, read);
                    handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0, new AsyncCallback(readSocketCallback), state);
                }
                else
                {
                    Console.WriteLine("Socket closed");
                    state.workSocket.Close();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error : Socket Error in readSocketCallback().!\nDetails : " + ex.ToString());
            }

        }
    }
}
