﻿using AviationCalcUtilNet.Geo;
using AviationCalcUtilNet.GeoTools;
using AviationCalcUtilNet.Magnetic;
using AviationCalcUtilNet.Units;
using NavData_Interface.DataSources;
using NavData_Interface.Objects;
using NavData_Interface.Objects.Fixes;
using NavData_Interface.Objects.Fixes.Waypoints;
using SaunaSim.Core.Simulator.Aircraft.FMS;
using SaunaSim.Core.Simulator.Aircraft.FMS.Legs;
using SaunaSim.Core.Simulator.Aircraft.FMS.VNAV;
using SaunaSim.Core.Simulator.Aircraft.Performance;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace sauna_tests
{
    [Explicit]
    public class FmsTests
    {
        private MagneticTileManager _magTileManager;

        [SetUp]
        public void Setup()
        {
            var model = MagneticModel.FromFile("WMM.COF");
            _magTileManager = new MagneticTileManager(ref model);
        }

        [Test]
        public void TestVnav1()
        {
            var navDataInterface = new DFDSource("e_dfd_2411.s3db");
            var star = navDataInterface.GetStarByAirportAndIdentifier("KBOS", "ROBUC3");
            star.selectTransition("JFK");
            star.selectRunwayTransition("04R");

            IList<IRouteLeg> legs = LegFactory.RouteLegsFromNavDataLegs(star.GetEnumerator(), _magTileManager);

            PerfData perfData = PerfDataHandler.LookupForAircraft("A320");

            PerfInit init = new PerfInit()
            {
                ClimbKias = perfData.Climb_KIAS,
                ClimbMach = (int)(perfData.Climb_Mach * 100),
                CruiseKias = perfData.Cruise_KIAS,
                CruiseMach = (int)(perfData.Cruise_Mach * 100),
                DescentKias = perfData.Descent_KIAS,
                DescentMach = (int)(perfData.Descent_Mach * 100),
                CruiseAlt = 35000,
                LimitAlt = 10000,
                LimitSpeed = 250,

                // TODO: Change to be based on Departure/Arrival airport
                TransitionAlt = 18000,
                TransitionLevel = 18000
            };

            var iterator = new FmsVnavLegIterator
            {
                Index = legs.Count - 1, // Start at last leg
                ApchAngle = null, // Approach angle (Only used in the approach phase)
                DistanceToRwy = null, // Distance to the runway threshold
                ShouldRewind = false,
                AlongTrackDistance = Length.FromMeters(0),
                LimitCrossed = false,

                // Information from last iteration
                LastAlt = null, // Altitude last waypoint was crossed at
                LastSpeed = -1, // Target speed last waypoint was crossed at
                LaterDecelLength = null, // Decel length left

                // Constraints from earlier (further up the arrival)
                EarlyUpperAlt = null,
                //EarlyLowerAlt = null,
                EarlySpeed = -1,

                // Index where constraints were detected
                EarlySpeedI = -2,
                EarlyUpperAltI = -2,
                //EarlyLowerAltI = -2,
            };

            // Loop through legs from last to first ending either when first leg is reached or cruise alt is reached
            while (iterator.Index > -1)
            {
                // Get current leg
                IRouteLeg curLeg = legs[iterator.Index];

                iterator = FmsVnavUtil.ProcessLegForVnav(curLeg, iterator, perfData, perfData.MTOW_kg, init, Length.FromFeet(0));
            }

            Console.WriteLine(legs);
        }
    }
}
