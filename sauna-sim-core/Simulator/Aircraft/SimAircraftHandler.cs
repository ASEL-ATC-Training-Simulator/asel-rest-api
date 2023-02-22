using System;
using System.Collections.Generic;

namespace SaunaSim.Core.Simulator.Aircraft
{
    public static class SimAircraftHandler
    {
        private static List<SimAircraft> _aircrafts;
        private static object _aircraftsLock = new object();
        private static bool _allPaused = true;

        static SimAircraftHandler()
        {
            lock (_aircraftsLock)
            {
                _aircrafts = new List<SimAircraft>();
            }
        }

        public static void PerformOnAircraft(Action<List<SimAircraft>> action)
        {
            lock (_aircraftsLock)
            {
                action(_aircrafts);
            }
        }

        public static bool AllPaused {
            get => _allPaused;
            set {
                _allPaused = value;
                lock (_aircraftsLock)
                {
                    foreach (SimAircraft aircraft in _aircrafts)
                    {
                        aircraft.Paused = _allPaused;
                    }
                }
            }
        }

        public static void RemoveAircraftByCallsign(string callsign)
        {
            lock (_aircraftsLock)
            {
                foreach (SimAircraft aircraft in _aircrafts)
                {
                    if (aircraft.Callsign.Equals(callsign))
                    {
                        aircraft.Dispose();
                        _aircrafts.Remove(aircraft);
                        break;
                    }
                }
            }
        }

        public static bool AddAircraft(SimAircraft aircraft)
        {
            lock (_aircraftsLock)
            {
                List<int> deletionList = new List<int>();

                for (int i = _aircrafts.Count - 1; i >= 0; i--)
                {
                    SimAircraft c = _aircrafts[i];
                    if (c.ConnectionStatus == ConnectionStatusType.DISCONNECTED || (c.Callsign.Equals(aircraft.Callsign) && c.ConnectionStatus == ConnectionStatusType.WAITING))
                    {
                        _aircrafts[i].Dispose();
                        _aircrafts.RemoveAt(i);
                    } else if (c.Callsign.Equals(aircraft.Callsign))
                    {
                        return false;
                    }
                }

                _aircrafts.Add(aircraft);
            }
            return true;
        }

        public static SimAircraft GetAircraftWhichContainsCallsign(string callsignMatch)
        {
            lock (_aircraftsLock)
            {
                foreach (SimAircraft aircraft in _aircrafts)
                {
                    if (aircraft.Callsign.ToLower().Contains(callsignMatch.ToLower()))
                    {
                        return aircraft;
                    }
                }
            }

            return null;
        }

        public static SimAircraft GetAircraftByCallsign(string callsign)
        {
            lock (_aircraftsLock)
            {
                foreach (SimAircraft aircraft in _aircrafts)
                {
                    if (aircraft.Callsign.ToLower().Equals(callsign.ToLower()))
                    {
                        return aircraft;
                    }
                }
            }

            return null;
        }

        public static void DeleteAllAircraft()
        {
            lock (_aircraftsLock)
            {
                foreach (SimAircraft aircraft in _aircrafts)
                {
                    aircraft.Dispose();
                }
                _aircrafts.Clear();
            }
        }
    }
}
