﻿using NavData_Interface.Objects.Fixes;
using NavData_Interface.Objects.Fixes.Waypoints;
using NavData_Interface.Objects.LegCollections.Procedures;
using SaunaSim.Core.Data;
using SaunaSim.Core.Simulator.Aircraft;
using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;

namespace SaunaSim.Core.Simulator.Commands
{
    public class SidCommand : IAircraftCommand
    {
        public SimAircraft Aircraft { get; set; }
        public Action<string> Logger { get; set; }

        private Sid _selectedSid;

        public void ExecuteCommand()
        {
            
        }

        public bool HandleCommand(ref List<string> args)
        {
            try
            {
                var runway = args[0];
                var sid = args[1];
                var transition = args[2];

                var airport = Aircraft.Fms.DepartureAirport;

                var foundSid = DataHandler.GetSidByAirportAndIdentifier(airport, sid);
                
                if (runway != "-")
                {
                    foundSid.selectRunwayTransition(runway);
                }
                if (transition != "-") 
                {
                    foundSid.selectTransition(transition);
                }
                _selectedSid = foundSid;

                Logger?.Invoke($"Will fly SID {sid} from runway {runway} and with transition {transition}");
                return true;
            } catch (Exception) {
                Logger?.Invoke($"Error");
                return false;
            }
        }
    }
}
