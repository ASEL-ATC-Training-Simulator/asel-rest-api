using System;
using System.Collections.Generic;
using System.Text;

namespace SaunaSim.Core.Simulator.Aircraft.Performance
{
    public enum EngineType
    {
        TURBOFAN,
        TURBOPROP,
        TURBO_PISTON,
        NA_PISTON,
    }
    public class PerfData
    {
        public int MTOW_kg { get; set; }
        public int MLW_kg { get; set; }
        public int OEW_kg { get; set; }
        public int MaxFuel_kg { get; set; }
        public int MZFW_kg { get; set; }
        public int Engines { get; set; }
        public EngineType EngineType { get; set; }
        public int IdleThrust_N { get; set; }
        public int MaxThrust_N { get; set; }
        public double WingArea_sqm { get; set; }
        public double WingSpan_m { get; set; }
        public double CrossSectionArea_sqm { get; set; }
        public List<ConfigSetting> ConfigList { get; set; }

        public class ConfigSetting
        {
            public int MinKias { get; set; }
            public int MaxKias { get; set; }
            public bool GearDown { get; set; }
            public int NormKias { get; set; }
        }

        // FMS Default Performance Data
        public int Climb_KIAS { get; set; }
        public double Climb_Mach { get; set; }
        public int Cruise_KIAS { get; set; }
        public double Cruise_Mach { get; set; }
        public int Descent_KIAS { get; set; }
        public double Descent_Mach { get; set; }
    }
}