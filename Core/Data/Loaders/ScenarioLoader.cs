using AviationCalcUtilNet.GeoTools;
using AviationCalcUtilNet.GeoTools.MagneticTools;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft.Control.FMS;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft.Control.FMS.Legs;
using VatsimAtcTrainingSimulator.Core.Simulator.Commands;

namespace VatsimAtcTrainingSimulator.Core.Data.Loaders
{
    public static class ScenarioLoader
    {
        public static void LoadSimulationScenario(string filename, Action<string> logMsg)
        {
            // Read file
            string[] filelines = File.ReadAllLines(filename);

            List<VatsimClientPilot> pilots = new List<VatsimClientPilot>();

            VatsimClientPilot lastPilot = null;

            foreach (string line in filelines)
            {
                // Create pilot and update position
                if (line.StartsWith("@N"))
                {
                    string[] items = line.Split(':');
                    string callsign = items[1];
                    XpdrMode xpdrMode = (XpdrMode)items[0].ToCharArray()[1];

                    lastPilot = new VatsimClientPilot(callsign, Properties.Settings.Default.cid, Properties.Settings.Default.password, "Simulator Pilot", Properties.Settings.Default.server, Properties.Settings.Default.port, Properties.Settings.Default.vatsimServer) {
                        Logger = (string msg) => {
                            logMsg($"{callsign}: {msg}");
                        }
                    };

                    // Send init position
                    lastPilot.SetInitialData(xpdrMode, Convert.ToInt32(items[2]), Convert.ToInt32(items[3]), Convert.ToDouble(items[4]), Convert.ToDouble(items[5]), Convert.ToDouble(items[6]), 250, Convert.ToInt32(items[8]), Convert.ToInt32(items[9]));

                    // Add to temp list
                    pilots.Add(lastPilot);
                } else if (line.StartsWith("$FP"))
                {
                    if (lastPilot != null)
                    {
                        lastPilot.FlightPlan = line;
                    }
                } else if (line.StartsWith("REQALT"))
                {

                    string[] items = line.Split(':');

                    if (lastPilot != null && items.Length >= 3)
                    {
                        try
                        {
                            int reqAlt = Convert.ToInt32(items[2]);
                            reqAlt /= 100;

                            List<string> args = new List<string>
                            {
                                    $"FL{reqAlt}"
                                };
                            CommandHandler.HandleCommand("dm", lastPilot, args, logMsg);
                        } catch (Exception) { }
                    }
                } else if (line.StartsWith("$ROUTE"))
                {
                    string[] items = line.Split(':');

                    if (lastPilot != null && items.Length >= 2)
                    {
                        string[] waypoints = items[1].Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                        List<IRouteLeg> legs = new List<IRouteLeg>();
                        FmsPoint lastPoint = null;


                        for (int i = 0; i < waypoints.Length; i++)
                        {
                            if (waypoints[i].ToLower() == "hold" && lastPoint != null)
                            {
                                PublishedHold pubHold = DataHandler.GetPublishedHold(lastPoint.Point.PointName);

                                if (pubHold != null)
                                {
                                    lastPoint.PointType = RoutePointTypeEnum.FLY_OVER;
                                    HoldToManualLeg leg = new HoldToManualLeg(lastPoint, BearingTypeEnum.MAGNETIC, pubHold.InboundCourse, pubHold.TurnDirection, pubHold.LegLengthType, pubHold.LegLength);
                                    legs.Add(leg);
                                    lastPoint = leg.EndPoint;
                                }
                            } else
                            {
                                Waypoint nextWp = DataHandler.GetClosestWaypointByIdentifier(waypoints[i], lastPilot.Position.Latitude, lastPilot.Position.Longitude);

                                if (nextWp != null)
                                {
                                    FmsPoint fmsPt = new FmsPoint(new RouteWaypoint(nextWp), RoutePointTypeEnum.FLY_BY);
                                    if (lastPoint == null)
                                    {
                                        lastPoint = fmsPt;
                                    } else
                                    {
                                        legs.Add(new TrackToFixLeg(lastPoint, fmsPt));
                                        lastPoint = fmsPt;
                                    }
                                }
                            }
                        }

                        foreach (IRouteLeg leg in legs)
                        {
                            lastPilot.Control.FMS.AddRouteLeg(leg);
                        }

                        if (legs.Count > 0)
                        {
                            lastPilot.Control.FMS.ActivateDirectTo(legs[0].StartPoint.Point);
                            LnavRouteInstruction instr = new LnavRouteInstruction();
                            lastPilot.Control.CurrentLateralInstruction = instr;
                        }
                    }
                } else if (line.StartsWith("START"))
                {
                    string[] items = line.Split(':');

                    if (lastPilot != null && items.Length >= 2)
                    {
                        try
                        {
                            int delay = Convert.ToInt32(items[1]) * 60000;
                            lastPilot.DelayMs = delay;
                        } catch (Exception) { }
                    }
                } else if (line.StartsWith("LOC"))
                {
                    // Format:  LOC:ID:Frequency:Course:Lat:Lon:DME Channel:DME Lat:DME Lon:Glidepath End Point Lat:Lon:Alt:Angle
                    // Example: LOC:I-AG:109.9:247:543903N:0061427W:36X:

                } else if (line.StartsWith("APP"))
                {
                    // Format:  APP:Airport ICAO:Approach Type:Letter:Runway:Navaid Id:GlidePath End Point Lat:Lon:Alt:Glidepath Angle:Initial Approach Segments (IAFs):Final Approach Segment (FAF onwards):Missed Approach Segments
                    // Example: APP:EGAA:ILS:Z:25:IAG:
                    string[] items = line.Split(':');
                    string type = items[1];
                } else if (line.StartsWith("HOLDING"))
                {
                    string[] items = line.Split(':');

                    try
                    {
                        string wpId = items[1];
                        double inboundCourse = Convert.ToDouble(items[2]);
                        HoldTurnDirectionEnum turnDirection = (HoldTurnDirectionEnum)Convert.ToInt32(items[3]);

                        DataHandler.AddPublishedHold(new PublishedHold(wpId, inboundCourse, turnDirection));
                    } catch (Exception)
                    {
                        Console.WriteLine("Could not load hold.");
                    }
                }
            }

            foreach (VatsimClientPilot pilot in pilots)
            {
                ClientsHandler.AddClient(pilot);
                pilot.ShouldSpawn = true;
            }
        }

        public static void LoadEuroscopeScenario(string filename, Action<string> logMsg)
        {
            // Read file
            string[] filelines = File.ReadAllLines(filename);

            List<VatsimClientPilot> pilots = new List<VatsimClientPilot>();

            VatsimClientPilot lastPilot = null;

            foreach (string line in filelines)
            {
                // Create pilot and update position
                if (line.StartsWith("@N"))
                {
                    string[] items = line.Split(':');
                    string callsign = items[1];
                    XpdrMode xpdrMode = (XpdrMode)items[0].ToCharArray()[1];

                    lastPilot = new VatsimClientPilot(callsign, Properties.Settings.Default.cid, Properties.Settings.Default.password, "Simulator Pilot", Properties.Settings.Default.server, Properties.Settings.Default.port, Properties.Settings.Default.vatsimServer) {
                        Logger = (string msg) => {
                            logMsg($"{callsign}: {msg}");
                        }
                    };

                    // Send init position
                    lastPilot.SetInitialData(xpdrMode, Convert.ToInt32(items[2]), Convert.ToInt32(items[3]), Convert.ToDouble(items[4]), Convert.ToDouble(items[5]), Convert.ToDouble(items[6]), 250, Convert.ToInt32(items[8]), Convert.ToInt32(items[9]));

                    // Add to temp list
                    pilots.Add(lastPilot);
                } else if (line.StartsWith("$FP"))
                {
                    if (lastPilot != null)
                    {
                        lastPilot.FlightPlan = line;
                    }
                } else if (line.StartsWith("REQALT"))
                {

                    string[] items = line.Split(':');

                    if (lastPilot != null && items.Length >= 3)
                    {
                        try
                        {
                            int reqAlt = Convert.ToInt32(items[2]);
                            reqAlt /= 100;

                            List<string> args = new List<string>
                            {
                                    $"FL{reqAlt}"
                                };
                            CommandHandler.HandleCommand("dm", lastPilot, args, logMsg);
                        } catch (Exception) { }
                    }
                } else if (line.StartsWith("$ROUTE"))
                {
                    string[] items = line.Split(':');

                    if (lastPilot != null && items.Length >= 2)
                    {
                        string[] waypoints = items[1].Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                        List<IRouteLeg> legs = new List<IRouteLeg>();
                        FmsPoint lastPoint = null;


                        for (int i = 0; i < waypoints.Length; i++)
                        {
                            Waypoint nextWp = DataHandler.GetClosestWaypointByIdentifier(waypoints[i], lastPilot.Position.Latitude, lastPilot.Position.Longitude);

                            if (nextWp != null)
                            {
                                FmsPoint fmsPt = new FmsPoint(new RouteWaypoint(nextWp), RoutePointTypeEnum.FLY_BY);
                                if (lastPoint == null)
                                {
                                    lastPoint = fmsPt;
                                } else
                                {
                                    legs.Add(new TrackToFixLeg(lastPoint, fmsPt));
                                    lastPoint = fmsPt;
                                }
                            }

                        }

                        foreach (IRouteLeg leg in legs)
                        {
                            lastPilot.Control.FMS.AddRouteLeg(leg);
                        }

                        if (legs.Count > 0)
                        {
                            lastPilot.Control.FMS.ActivateDirectTo(legs[0].StartPoint.Point);
                            LnavRouteInstruction instr = new LnavRouteInstruction();
                            lastPilot.Control.CurrentLateralInstruction = instr;
                        }
                    }
                } else if (line.StartsWith("START"))
                {
                    string[] items = line.Split(':');

                    if (lastPilot != null && items.Length >= 2)
                    {
                        try
                        {
                            int delay = Convert.ToInt32(items[1]) * 60000;
                            lastPilot.DelayMs = delay;
                        } catch (Exception) { }
                    }
                } else if (line.StartsWith("ILS"))
                {
                    string[] items = line.Split(':');
                    string wpId = items[0];

                    try
                    {
                        GeoPoint threshold = new GeoPoint(Convert.ToDouble(items[1]), Convert.ToDouble(items[2]));
                        double course = 0;
                        if (items.Length == 4)
                        {
                            course = Convert.ToDouble(items[3]);
                        } else if (items.Length > 4)
                        {
                            GeoPoint otherThreshold = new GeoPoint(Convert.ToDouble(items[3]), Convert.ToDouble(items[4]));
                            course = MagneticUtil.ConvertTrueToMagneticTile(GeoPoint.InitialBearing(threshold, otherThreshold), threshold);
                        }

                        DataHandler.AddWaypoint(new Localizer(wpId, threshold.Lat, threshold.Lon, wpId, 0, course));
                    } catch (Exception)
                    {
                        Console.WriteLine("Well that didn't work did it.");
                    }
                } else if (line.StartsWith("HOLDING"))
                {
                    string[] items = line.Split(':');

                    try
                    {
                        string wpId = items[1];
                        double inboundCourse = Convert.ToDouble(items[2]);
                        HoldTurnDirectionEnum turnDirection = (HoldTurnDirectionEnum)Convert.ToInt32(items[3]);

                        DataHandler.AddPublishedHold(new PublishedHold(wpId, inboundCourse, turnDirection));
                    } catch (Exception)
                    {
                        Console.WriteLine("Well that didn't work did it.");
                    }
                }
            }

            foreach (VatsimClientPilot pilot in pilots)
            {
                ClientsHandler.AddClient(pilot);
                pilot.ShouldSpawn = true;
            }
        }
    }
}
