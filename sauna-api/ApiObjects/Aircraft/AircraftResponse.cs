using SaunaSim.Core;
using SaunaSim.Core.Data;
using SaunaSim.Core.Simulator.Aircraft;
using SaunaSim.Core.Simulator.Aircraft.Control;
using SaunaSim.Core.Simulator.Aircraft.Control.FMS;
using SaunaSim.Core.Simulator.Aircraft.Control.FMS.Legs;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using System.Collections.Generic;
using FsdConnectorNet;

namespace SaunaSim.Api.ApiObjects.Aircraft
{
    public class AircraftResponse
    {
        public string Callsign { get; set; }
        public int DelayMs { get; set; }
        public bool Paused { get; set; }
        public string FlightPlan { get; set; }
        public double ThrustLeverPos { get; set; }
        public TransponderModeType XpdrMode { get; set; }
        public int Squawk { get; set; }
        public AircraftPosition Position { get; set; }
        public AircraftControlResponse Control { get; set; }
        public int Assigned_IAS { get; set; }
        public ConstraintType Assigned_IAS_Type { get; set; }
        public ConnectionStatusType ConnectionStatus { get; set; }

        public AircraftResponse()
        {

        }

        public AircraftResponse(SimAircraft pilot, bool includeFms = false)
        {
            Callsign = pilot.Callsign;
            DelayMs = pilot.DelayMs;
            Paused = pilot.Paused;
            FlightPlan = pilot.FlightPlan;
            XpdrMode = pilot.XpdrMode;
            Squawk = pilot.Squawk;
            Position = pilot.Position;
            Control = new AircraftControlResponse(pilot.Control, includeFms);
            Assigned_IAS = pilot.Assigned_IAS;
            Assigned_IAS_Type = pilot.Assigned_IAS_Type;
            ConnectionStatus = pilot.ConnectionStatus;
            ThrustLeverPos = pilot.ThrustLeverPosition;
        }

        public class AircraftControlResponse
        {
            public object CurrentLateralMode { get; set; }
            public string CurrentLateralModeStr { get; set; }
            public object ArmedLateralMode { get; set; }
            public string ArmedLateralModeStr { get; set; }
            public object CurrentVerticalMode { get; set; }
            public string CurrentVerticalModeStr { get; set; }
            public List<object> ArmedVerticalModes { get; set; }
            public List<string> ArmedVerticalModesStr { get; set; }
            public AircraftFmsResponse FMS { get; set; }

            public AircraftControlResponse()
            {

            }

            public AircraftControlResponse(AircraftControl control, bool includeFms = false)
            {
                CurrentLateralMode = control.CurrentLateralInstruction;
                CurrentLateralModeStr = control.CurrentLateralInstruction?.ToString();
                ArmedLateralMode = control.ArmedLateralInstruction;
                ArmedLateralModeStr = control.ArmedLateralInstruction?.ToString();
                CurrentVerticalMode = control.CurrentVerticalInstruction;
                CurrentVerticalModeStr = control.CurrentVerticalInstruction?.ToString();
                ArmedVerticalModes = new List<object>();
                ArmedVerticalModesStr = new List<string>();
                foreach (var instr in control.ArmedVerticalInstructions)
                {
                    ArmedVerticalModes.Add(instr);
                    ArmedVerticalModesStr.Add(instr.ToString());
                }
                if (includeFms)
                {
                    FMS = new AircraftFmsResponse(control.FMS);
                }
            }
        }
    }
}