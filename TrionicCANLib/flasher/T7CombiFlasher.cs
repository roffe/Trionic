using System;
using System.Collections.Generic;
using System.Diagnostics;
using Combi;
using NLog;

namespace TrionicCANLib.Flasher
{
    //-------------------------------------------------------------------------------------------------
    public class T7CombiFlasher : IFlasher
    {
        // dynamic state
        private readonly caCombiAdapter combi;       ///< adapter object

        // events
        public override event IFlasher.StatusChanged onStatusChanged;

        private Logger logger = LogManager.GetCurrentClassLogger();

        // errors go to the GUI log too, otherwise the user only sees the generic failure
        private void Notify(string info)
        {
            logger.Debug(info);
            onStatusChanged?.Invoke(this, new StatusEventArgs(info));
        }

        //---------------------------------------------------------------------------------------------
        /**
            Constructor.
          
            @param      _combi      adapter object
        */
        public T7CombiFlasher(caCombiAdapter _combi)
        {
            Debug.Assert(_combi != null);
            combi = _combi;
        }

        //---------------------------------------------------------------------------------------------
        /**
            Returns the current status of flasher.
         
            @return                 status 
        */
        public override FlashStatus getStatus()
        {
            if (combi.OperationRunning() || m_flashStatus == FlashStatus.StartingFlashSession ||
                m_flashStatus == FlashStatus.ReadError || m_flashStatus == FlashStatus.WriteError)
            {
                // in progress, still connecting to the ECU (no operation running yet, nothing failed),
                // or failed before the operation could start
                return m_flashStatus;
            }

            if (!combi.OperationSucceeded())
            {
                // failed (no way to know the real reason)
                switch (m_command)
                {
                    case FlashCommand.ReadCommand:
                    case FlashCommand.ReadMemoryCommand:
                    case FlashCommand.ReadSymbolMapCommand:
                    case FlashCommand.ReadSymbolNameCommand:
                        return FlashStatus.ReadError;

                    case FlashCommand.WriteCommand:
                        return FlashStatus.WriteError;
                };
            }

            return FlashStatus.Completed;
        }

        //---------------------------------------------------------------------------------------------
        /**
            Returns the number of bytes that has been read or written so far; 0 is 
            returned if there is no read or write session ongoing.
         
            @return                 number of bytes 
        */
        public override int getNrOfBytesRead()
        {
            return (int)combi.GetOperationProgress();
        }

        //---------------------------------------------------------------------------------------------
        /**
            Interrupts ongoing read or write session.
        */
        public override void stopFlasher()
        {
            endSession();

            base.stopFlasher();
            m_flashStatus = FlashStatus.Completed;
        }

        //---------------------------------------------------------------------------------------------
        /**
            Starts a flash reading session.
            
            @param      a_fileName      name of target file
        */
        public override void readFlash(string a_fileName)
        {
            m_flashStatus = FlashStatus.StartingFlashSession;
            base.readFlash(a_fileName);

            try
            {
                // connect to ECU; this may take some time as both
                // P-Bus and I-bus are checked for traffic
                if (!beginSession())
                {
                    throw new Exception("Failed to start session");
                }

                // read flash    
                combi.CAN_ReadFlash(a_fileName);
                m_flashStatus = FlashStatus.Reading;
            }

            catch (Exception e)
            {
                Notify("Read error: " + e.Message);
                m_flashStatus = FlashStatus.ReadError;
            }
        }

        //---------------------------------------------------------------------------------------------
        /**
            Starts a flash writing session.
            
            @param      a_fileName      name of target file
        */
        public override void writeFlash(string a_fileName)
        {
            m_flashStatus = FlashStatus.StartingFlashSession;
            base.writeFlash(a_fileName);

            try
            {
                // connect to ECU; this may take some time as both
                // P-Bus and I-bus are checked for traffic
                if (!beginSession())
                {
                    throw new Exception("Failed to start session");
                }

                // read flash    
                combi.CAN_WriteFlash(a_fileName, 0);
                m_flashStatus = FlashStatus.Writing;
            }

            catch (Exception e)
            {
                Notify("Write error: " + e.Message);
                m_flashStatus = FlashStatus.WriteError;
            }
        }

        //---------------------------------------------------------------------------------------------
        /**
            Begins a communication session with ECU.
          
            @return             succ / fail
        */
        public bool beginSession()
        {
            try
            {
                combi.CAN_ConnectECU(3);
                return true;
            }

            catch (Exception e)
            {
                Notify("Session error: " + e.Message);
                return false;
            }
        }

        //---------------------------------------------------------------------------------------------
        /**
            Ends a communication session with ECU.     
        */
        public void endSession()
        {
            logger.Debug("End communication session");
            combi.CAN_DisconnectECU(false);
        }

        public override void cleanup()
        {
        }
    };
}
