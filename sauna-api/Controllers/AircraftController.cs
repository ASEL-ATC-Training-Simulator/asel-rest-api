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
using SaunaSim.Core.Simulator.Aircraft;
using System.Runtime.CompilerServices;
using SaunaSim.Api.Utilities;
using SaunaSim.Core.Simulator.Aircraft.Autopilot.Controller;
using SaunaSim.Core.Simulator.Aircraft.FMS;
using SaunaSim.Core.Simulator.Aircraft.FMS.Legs;
using SaunaSim.Core.Simulator.Aircraft.Performance;
using NavData_Interface.Objects.Fix;
using SaunaSim.Core.Data.Loaders;
using SaunaSim.Core.Simulator.Aircraft.Autopilot;
using System.Threading;
using SaunaSim.Api.WebSockets;

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
        public ActionResult<AircraftResponse> CreateAircraft(CreateAircraftRequest request)
        {
            try
            {
                AircraftBuilder builder = new AircraftBuilder(
                    request.Callsign,
                    request.Cid,
                    request.Password,
                    request.Server,
                    request.Port)
                {
                    FullName = request.FullName,
                    Protocol = request.Protocol,
                    Position = new AviationCalcUtilNet.GeoTools.GeoPoint(request.Position.Latitude,
                    request.Position.Longitude,
                    request.Position.IndicatedAltitude),
                    HeadingMag = request.Position.MagneticHeading,
                    LogInfo = (string msg) =>
                    {
                        _logger.LogInformation($"{request.Callsign}: {msg}");
                    },
                    LogWarn = (string msg) =>
                    {
                        _logger.LogWarning($"{request.Callsign}: {msg}");
                    },
                    LogError = (string msg) =>
                    {
                        _logger.LogError($"{request.Callsign}: {msg}");
                    },
                    XpdrMode = request.TransponderMode,
                    Squawk = request.Squawk,
                    Speed = request.Position.IndicatedSpeed,
                    IsSpeedMach = request.Position.IsMachNumber
                };

                if (request.FmsWaypointList != null)
                {
                    builder.FmsWaypoints = request.FmsWaypointList;
                }

                var pilot = builder.Push(PrivateInfoLoader.GetClientInfo((string msg) => { _logger.LogWarning($"{request.Callsign}: {msg}"); }));

                return Ok(new AircraftResponse(pilot, true));
            } catch (Exception e)
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

        [HttpGet("websocketByCallsign/{callsign}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public async Task WebSocketForCallsign(string callsign)
        {
            if (SimAircraftHandler.GetAircraftByCallsign(callsign) == null)
            {
                HttpContext.Response.StatusCode = StatusCodes.Status400BadRequest;
            } else if (HttpContext.WebSockets.IsWebSocketRequest)
            {
                try
                {
                    using var webSocket = await HttpContext.WebSockets.AcceptWebSocketAsync();
                    await WebSocketHandler.HandleAircraftSocket(callsign, webSocket);
                } catch (Exception e)
                {
                    _logger.LogWarning($"Websocket connection failed: {e.Message}");
                }
            } else
            {
                HttpContext.Response.StatusCode = StatusCodes.Status400BadRequest;
            }
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

        [HttpPost("all/unpause")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult<AircraftStateRequestResponse> UnpauseAll()
        {
            SimAircraftHandler.AllPaused = false;

            return Ok(new AircraftStateRequestResponse
            {
                Paused = SimAircraftHandler.AllPaused,
                SimRate = SimAircraftHandler.SimRate
            });
        }

        [HttpPost("all/pause")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult<AircraftStateRequestResponse> PauseAll()
        {
            SimAircraftHandler.AllPaused = true;

            return Ok(new AircraftStateRequestResponse
            {
                Paused = SimAircraftHandler.AllPaused,
                SimRate = SimAircraftHandler.SimRate
            });
        }

        [HttpPost("all/simrate")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult<AircraftStateRequestResponse> SetAllSimRate(AircraftStateRequestResponse request)
        {
            SimAircraftHandler.SimRate = request.SimRate;

            return Ok(new AircraftStateRequestResponse
            {
                Paused = SimAircraftHandler.AllPaused,
                SimRate = SimAircraftHandler.SimRate
            });
        }

        [HttpGet("all/simState")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult<AircraftStateRequestResponse> GetAllSimState()
        {
            return Ok(new AircraftStateRequestResponse
            {
                Paused = SimAircraftHandler.AllPaused,
                SimRate = SimAircraftHandler.SimRate
            });
        }

        [HttpPost("byCallsign/{callsign}/unpause")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftStateRequestResponse> UnpauseAircraft(string callsign)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Paused = false;

            return Ok(new AircraftStateRequestResponse
            {
                Paused = client.Paused,
                SimRate = client.SimRate / 10.0
            });
        }

        [HttpPost("byCallsign/{callsign}/pause")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftStateRequestResponse> PauseAircraft(string callsign)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Paused = true;

            return Ok(new AircraftStateRequestResponse
            {
                Paused = client.Paused,
                SimRate = client.SimRate / 10.0
            });
        }

        [HttpPost("byCallsign/{callsign}/simrate")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftStateRequestResponse> SetAircraftSimRate(string callsign, AircraftStateRequestResponse request)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.SimRate = (int)(request.SimRate * 10);

            return Ok(new AircraftStateRequestResponse
            {
                Paused = client.Paused,
                SimRate = client.SimRate / 10.0
            });
        }

        [HttpGet("byCallsign/{callsign}/simState")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftStateRequestResponse> GetAircraftSimState(string callsign)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            return Ok(new AircraftStateRequestResponse
            {
                Paused = SimAircraftHandler.AllPaused,
                SimRate = SimAircraftHandler.SimRate
            });
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
        [ProducesResponseType(StatusCodes.Status200OK)]
        public ActionResult RemoveAll()
        {
            SimAircraftHandler.DeleteAllAircraft();
            return Ok();
        }

        [HttpPost("mcp/byCallsign/{callsign}/setSpeedMode/{speedMode}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftAutopilot> McpSetSpeedMode(string callsign, McpSpeedSelectorType speedMode)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Autopilot.SelectedSpeedMode = speedMode;

            return Ok(client.Autopilot);
        }

        [HttpPost("mcp/byCallsign/{callsign}/setSpeedUnits/{speedUnits}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftAutopilot> McpSetSpeedUnits(string callsign, McpSpeedUnitsType speedUnits)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Autopilot.SelectedSpeedUnits = speedUnits;

            return Ok(client.Autopilot);
        }

        [HttpPost("mcp/byCallsign/{callsign}/setSelSpeed/{selSpeed}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftAutopilot> McpSetSpeedBug(string callsign, int selSpeed)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Autopilot.SelectedSpeed = selSpeed;

            return Ok(client.Autopilot);
        }

        [HttpPost("mcp/byCallsign/{callsign}/setSelHdg/{selHdg}/{turnDir}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftAutopilot> McpSetHdgBug(string callsign, int selHdg, McpKnobDirection turnDir)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Autopilot.SelectedHeading = selHdg;
            client.Autopilot.HdgKnobTurnDirection = turnDir;

            return Ok(client.Autopilot);
        }

        [HttpPost("mcp/byCallsign/{callsign}/setSelAlt/{selAlt}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftAutopilot> McpSetAltBug(string callsign, int selAlt)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Autopilot.SelectedAltitude = selAlt;

            return Ok(client.Autopilot);
        }

        [HttpPost("mcp/byCallsign/{callsign}/setSelFpa/{selFpa}")]
        [ProducesResponseType(StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        public ActionResult<AircraftAutopilot> McpSetFpaBug(string callsign, int selFpa)
        {
            SimAircraft client = SimAircraftHandler.GetAircraftByCallsign(callsign);

            if (client == null)
            {
                return BadRequest("The aircraft was not found!");
            }

            client.Autopilot.SelectedFpa = selFpa;

            return Ok(client.Autopilot);
        }
    }
}
