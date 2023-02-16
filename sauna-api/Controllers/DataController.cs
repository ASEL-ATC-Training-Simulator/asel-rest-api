﻿using SaunaSim.Api.ApiObjects.Data;
using SaunaSim.Core;
using SaunaSim.Core.Data;
using SaunaSim.Core.Simulator.Aircraft;
using SaunaSim.Core.Simulator.Aircraft.Control.FMS;
using SaunaSim.Core.Simulator.Aircraft.Control.FMS.Legs;
using SaunaSim.Core.Simulator.Commands;
using AviationCalcUtilNet.GeoTools;
using AviationCalcUtilNet.GeoTools.MagneticTools;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using VatsimAtcTrainingSimulator;

namespace SaunaSim.Api.Controllers
{
    [ApiController]
    [Route("api/data")]
    public class DataController : ControllerBase
    {

        private readonly ILogger<DataController> _logger;

        public DataController(ILogger<DataController> logger)
        {
            _logger = logger;
        }

        [HttpGet("settings")]
        public AppSettings GetSettings()
        {
            return AppSettingsManager.Settings;
        }

        [HttpPost("settings")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AppSettings> UpdateSettings(AppSettings settings)
        {
            AppSettingsManager.Settings = settings;

            return Ok(AppSettingsManager.Settings);
        }

        [HttpPost("loadMagneticFile")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<string> LoadMagneticFile()
        {
            try
            {
                MagneticUtil.LoadData();
            } catch (Exception)
            {
                return BadRequest("There was an error loading the WMM.COF file. Ensure that WMM.COF is placed in the 'magnetic' folder.");
            }
            return Ok("Magnetic File Loaded");
        }

        [HttpPost("loadSectorFile")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult LoadSectorFile(LoadFileRequest request)
        {
            // Read file
            string filename = request.FileName;
            try
            {
                string[] filelines = System.IO.File.ReadAllLines(filename);

                string sectionName = "";

                // Loop through sector file
                foreach (string line in filelines)
                {
                    // Ignore comments
                    if (line.Trim().StartsWith(";"))
                    {
                        continue;
                    }

                    if (line.StartsWith("["))
                    {
                        // Get section name
                        sectionName = line.Replace("[", "").Replace("]", "").Trim();
                    } else
                    {
                        NavaidType type = NavaidType.VOR;
                        string[] items;
                        switch (sectionName)
                        {
                            case "VOR":
                                type = NavaidType.VOR;
                                goto case "AIRPORT";
                            case "NDB":
                                type = NavaidType.NDB;
                                goto case "AIRPORT";
                            case "AIRPORT":
                                type = NavaidType.AIRPORT;

                                items = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                                if (items.Length >= 4)
                                {
                                    decimal freq = 0;
                                    try
                                    {
                                        freq = Convert.ToDecimal(items[1]);
                                    } catch (Exception) { }

                                    GeoUtil.ConvertVrcToDecimalDegs(items[2], items[3], out double lat, out double lon);
                                    DataHandler.AddWaypoint(new WaypointNavaid(items[0], lat, lon, "", freq, type));
                                }
                                break;
                            case "FIXES":
                                items = line.Split(' ');

                                if (items.Length >= 3)
                                {
                                    GeoUtil.ConvertVrcToDecimalDegs(items[1], items[2], out double lat, out double lon);
                                    DataHandler.AddWaypoint(new Waypoint(items[0],  lat, lon));
                                }
                                break;
                        }
                    }
                }
            } catch (Exception ex)
            {
                return BadRequest(ex);
            }
            return Ok();
        }

        [HttpPost("loadEuroscopeScenario")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult LoadEuroscopeScenario(LoadScenarioFileRequest request)
        {
            try
            {
                string[] filelines = System.IO.File.ReadAllLines(request.FileName);

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

                        lastPilot = new VatsimClientPilot(callsign, request.Cid, request.Password, "Simulator Pilot", request.Server, request.Port, request.VatsimServer, request.Protocol) {
                            Logger = (string msg) => {
                                _logger.LogInformation($"{callsign}: {msg}");
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
                                CommandHandler.HandleCommand("dm", lastPilot, args, (string msg) => _logger.LogInformation(msg));
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
            } catch (Exception ex)
            {
                return BadRequest(ex);
            }
            return Ok();
        }

    }
}
