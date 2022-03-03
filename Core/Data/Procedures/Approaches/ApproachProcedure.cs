using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft.Control.FMS;

namespace VatsimAtcTrainingSimulator.Core.Data.Procedures.Approaches
{
    public class ApproachTypeAttr : Attribute
    {
        public ApproachTypeAttr(string shortName, string longName, string levelDName)
        {
            ShortName = shortName;
            LongName = longName;
            LevelDName = levelDName;
        }

        public string ShortName { get; private set; }
        public string LongName { get; private set; }
        public string LevelDName { get; private set; }
    }

    public static class ApproachTypes
    {
        public static string GetShortName(this ApproachType type)
        {
            return GetAttr(type).ShortName;
        }

        public static string GetLongName(this ApproachType type)
        {
            return GetAttr(type).LongName;
        }

        public static string GetLevelDName(this ApproachType type)
        {
            return GetAttr(type).LevelDName;
        }

        private static ApproachTypeAttr GetAttr(ApproachType p)
        {
            return (ApproachTypeAttr)Attribute.GetCustomAttribute(ForValue(p), typeof(ApproachTypeAttr));
        }

        private static MemberInfo ForValue(ApproachType p)
        {
            return typeof(ApproachType).GetField(Enum.GetName(typeof(ApproachType), p));
        }
    }

    public enum ApproachType
    {
        [ApproachTypeAttr("ILS", "Instrument Landing System", "ILS")] ILS,
        [ApproachTypeAttr("GLS", "GBAS Landing System", "GLS")] GLS,
        [ApproachTypeAttr("LOC", "Localizer", "LOC")] LOC,
        [ApproachTypeAttr("RNAV", "Area Navigation", "RNV")] RNAV,
        [ApproachTypeAttr("RNP", "Required Navigation Precision", "RNP")] RNP,
        [ApproachTypeAttr("GPS", "Global Positioning System", "GPS")] GPS,
        [ApproachTypeAttr("VOR", "VHF Omnidirectional Range", "VOR")] VOR,
        [ApproachTypeAttr("NDB", "Non-directional Beacon", "NDB")] NDB,
        [ApproachTypeAttr("LDA", "Localizer-Type Directional Aid", "LDA")] LDA
    }

    public class ApproachProcedure
    {
        private ApproachType _type;
        private char _letter;
        private string _rwy;
        private string _airportIcao;
        private string _navaidId;
        private ApproachMinimums _mins;
        private ApproachMinimums _circleMins;
        private List<ProcedureSegment> _initSegments;
        private ProcedureSegment _finalSegment;
        private List<ProcedureSegment> _maSegments;

        private ApproachProcedure()
        {
            _type = ApproachType.ILS;
            _initSegments = new List<ProcedureSegment>();
            _finalSegment = new ProcedureSegment();

        }
        private ApproachProcedure(string airportIcao, ApproachType type, char letter, string rwy, ApproachMinimums mins, ApproachMinimums circleMins, List<ProcedureSegment> initSegments, ProcedureSegment finalSegment, ProcedureSegment defaultMaSegment, List<ProcedureSegment> alternateMaSegments)
        {
            _airportIcao = airportIcao;
            _type = type;
            _letter = letter;
            _rwy = rwy;
            _mins = mins;
            _circleMins = circleMins;
            _initSegments = initSegments;
            _finalSegment = finalSegment;
            _maSegments = new List<ProcedureSegment>
            {
                defaultMaSegment
            };
            _maSegments.AddRange(alternateMaSegments);
        }

        public string AirportIcao {
            get => _airportIcao;
            set => _airportIcao = value;
        }

        public string NavaidId {
            get => _navaidId;
            set => _navaidId = value;
        }

        public ApproachType Type
        {
            get => _type;
            set => _type = value;
        }

        public char Letter
        {
            get => _letter;
            set => _letter = value;
        }

        public string Runway
        {
            get => _rwy;
            set => _rwy = value;
        }

        public string ShortName => $"{Type.GetShortName()}{Letter}{Runway}";

        public string LevelDName => $"{Type.GetLevelDName()}{Letter}{Runway}";

        public ApproachMinimums Minimums {
            get => _mins;
            set => _mins = value;
        }

        public ApproachMinimums CirclingMinimums
        {
            get => _circleMins;
            set => _circleMins = value;
        }

        public List<ProcedureSegment> InitialSegments {
            get => _initSegments;
            set => _initSegments = value ?? new List<ProcedureSegment>();
        }

        public ProcedureSegment FinalSegment
        {
            get => _finalSegment;
            set => _finalSegment = value ?? new ProcedureSegment();
        }

        public List<ProcedureSegment> MissedApproachSegments
        {
            get => _maSegments;
            set => _maSegments = value ?? new List<ProcedureSegment>();
        }

        public ProcedureSegment DefaultMissedSegment => _maSegments.Count >= 1 ? _maSegments[0] : null;
    }
}
