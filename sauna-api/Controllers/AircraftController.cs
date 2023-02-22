using SaunaSim.Api.ApiObjects.Aircraft;
using SaunaSim.Core;
using SaunaSim.Core.Data;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using SaunaSim.Core.Simulator.Aircraft.Control.FMS.Legs;
using SaunaSim.Core.Simulator.Aircraft.Control.FMS;
using SaunaSim.Core.Simulator.Aircraft;
using System.Runtime.CompilerServices;
using SaunaSim.Api.Utilities;

namespace SaunaSim.Api.Controllers
{
    [ApiController]
    [Route("api/aircraft")]
    public class AircraftController : ControllerBase
    {
        private readonly ILogger<DataController> _logger;

        public AircraftController(ILogger<DataController> logger)
        {
            _logger = logger;
        }

        [HttpPost("create")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftResponse>
            CreateAircraft(CreateAircraftRequest request) // reuses code from DataController.LoadEuroScopeScenario. Both should use the same function.
        {
            try
            {
                SimAircraft pilot = new SimAircraft(request.Callsign, request.Cid, request.Password, request.FullName, request.Server, (ushort)request.Port, request.Protocol,
                    ClientInfoLoader.GetClientInfo((string msg) => { _logger.LogWarning($"{request.Callsign}: {msg}"); }),
                    request.Position.Latitude, request.Position.Longitude, request.Position.IndicatedAltitude, request.Position.MagneticHeading)
                {
                    LogInfo = (string msg) => { _logger.LogInformation($"{request.Callsign}: {msg}"); },
                    LogWarn = (string msg) => { _logger.LogWarning($"{request.Callsign}: {msg}"); },
                    LogError = (string msg) => { _logger.LogError($"{request.Callsign}: {msg}"); },

                    XpdrMode = request.TransponderMode,
                    Squawk = request.Squawk,
                    Paused = request.Paused,
                    FlightPlan = request.FlightPlan,
                };


                if (request.Position.IsMachNumber)
                {
                    pilot.Position.MachNumber = request.Position.IndicatedSpeed;
                }
                else
                {
                    pilot.Position.IndicatedAirSpeed = request.Position.IndicatedSpeed;
                }


                List<IRouteLeg> legs = new List<IRouteLeg>();

                FmsPoint lastPoint = null;

                foreach (FmsWaypointRequest waypoint in request.FmsWaypointList)
                {
                    if (waypoint.Identifier.ToLower() == "hold" && lastPoint != null)
                    {
                        PublishedHold pubHold = DataHandler.GetPublishedHold(lastPoint.Point.PointName);

                        if (pubHold != null)
                        {
                            lastPoint.PointType = RoutePointTypeEnum.FLY_OVER;
                            HoldToManualLeg leg = new HoldToManualLeg(lastPoint, BearingTypeEnum.MAGNETIC, pubHold.InboundCourse, pubHold.TurnDirection, pubHold.LegLengthType,
                                pubHold.LegLength);
                            legs.Add(leg);
                            lastPoint = leg.EndPoint;
                        }
                    }
                    else
                    {
                        Waypoint nextWp = DataHandler.GetClosestWaypointByIdentifier(waypoint.Identifier, pilot.Position.Latitude, pilot.Position.Longitude);

                        if (nextWp != null)
                        {
                            FmsPoint fmsPt = new FmsPoint(new RouteWaypoint(nextWp), RoutePointTypeEnum.FLY_BY)
                            {
                                UpperAltitudeConstraint = waypoint.UpperAltitudeConstraint,
                                LowerAltitudeConstraint = waypoint.LowerAltitudeConstraint,
                                SpeedConstraintType = waypoint.SpeedConstratintType,
                                SpeedConstraint = waypoint.SpeedConstraint,
                            };

                            if (lastPoint == null)
                            {
                                lastPoint = fmsPt;
                            }
                            else
                            {
                                legs.Add(new TrackToFixLeg(lastPoint, fmsPt));
                                lastPoint = fmsPt;
                            }
                        }
                    }
                }

                foreach (IRouteLeg leg in legs)
                {
                    pilot.Control.FMS.AddRouteLeg(leg);
                }

                if (legs.Count > 0)
                {
                    pilot.Control.FMS.ActivateDirectTo(legs[0].StartPoint.Point);
                    LnavRouteInstruction lnavInstr = new LnavRouteInstruction();
                    pilot.Control.CurrentLateralInstruction = lnavInstr;
                }

                AltitudeHoldInstruction altInstr = new AltitudeHoldInstruction((int)pilot.Position.IndicatedAltitude);
                pilot.Control.CurrentVerticalInstruction = altInstr;

                SimAircraftHandler.AddAircraft(pilot);
                pilot.Start();

                return Ok(new AircraftResponse(pilot, true));
            }
            catch (Exception e)
            {
                return BadRequest(e);
            }
        }

        [HttpGet("getAll")]
        public List<AircraftResponse> GetAllAircraft()
        {
            List<AircraftResponse> pilots = new List<AircraftResponse>();
            SimAircraftHandler.PerformOnAircraft((list =>
            {
                foreach (var pilot in list)
                {
                    pilots.Add(new AircraftResponse(pilot));
                }
            }));

            return pilots;
        }

        [HttpGet("getAllWithFms")]
        public List<AircraftResponse> GetAllAircraftWithFms()
        {
            List<AircraftResponse> pilots = new List<AircraftResponse>();
            SimAircraftHandler.PerformOnAircraft((list =>
            {
                foreach (var pilot in list)
                {
                    pilots.Add(new AircraftResponse(pilot, true));
                }
            }));

            return pilots;
        }

        [HttpGet("getByCallsign/{callsign}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftResponse> GetAircraftByCallsign(string callsign)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            return Ok(new AircraftResponse(client, true));
        }

        [HttpGet("getByPartialCallsign/{callsign}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftResponse> GetAircraftByPartialCallsign(string callsign)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftWhichContainsCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            return Ok(new AircraftResponse(client, true));
        }

        [HttpPost("all/pause")]
        public ActionResult PauseAll()
        {
            SimAircraftHandler.AllPaused = true;
            return Ok();
        }

        [HttpPost("all/unpause")]
        public ActionResult UnpauseAll()
        {
            SimAircraftHandler.AllPaused = false;
            return Ok();
        }

        [HttpDelete("removeByCallsign/{callsign}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftResponse> RemoveAircraftByCallsign(string callsign)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            SimAircraftHandler.RemoveAircraftByCallsign(callsign);

            return Ok(new AircraftResponse(client, true));
        }

        [HttpDelete("all/remove")]
        public ActionResult RemoveAll()
        {
            SimAircraftHandler.DeleteAllAircraft();
            return Ok();
        }
    }
}