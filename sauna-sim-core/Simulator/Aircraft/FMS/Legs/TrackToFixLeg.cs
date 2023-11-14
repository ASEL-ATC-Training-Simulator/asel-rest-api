﻿using System;
using System.Collections.Generic;
using AviationCalcUtilNet.GeoTools;
using SaunaSim.Core.Simulator.Aircraft.Autopilot.Controller;

namespace SaunaSim.Core.Simulator.Aircraft.FMS.Legs
{
    public class TrackToFixLeg : IRouteLeg
    {
        private FmsPoint _startPoint;
        private FmsPoint _endPoint;
        private double _initialBearing;
        private double _finalBearing;
        private double _prevAlongTrackDist;

        public TrackToFixLeg(FmsPoint startPoint, FmsPoint endPoint)
        {
            _startPoint = startPoint;
            _endPoint = endPoint;
            _initialBearing = GeoPoint.InitialBearing(_startPoint.Point.PointPosition, _endPoint.Point.PointPosition);
            _finalBearing = GeoPoint.FinalBearing(_startPoint.Point.PointPosition, _endPoint.Point.PointPosition);
        }

        public RouteLegTypeEnum LegType => RouteLegTypeEnum.TRACK_TO_FIX;

        public double InitialTrueCourse => _initialBearing;

        public double FinalTrueCourse => _finalBearing;

        public FmsPoint EndPoint => _endPoint;

        public FmsPoint StartPoint => _startPoint;

        public bool HasLegTerminated(SimAircraft aircraft)
        {
            // Leg terminates when aircraft passes abeam/over terminating point
            GeoUtil.CalculateCrossTrackErrorM(aircraft.Position.PositionGeoPoint, _endPoint.Point.PointPosition, _finalBearing,
                out _, out double alongTrackDistance);

            return alongTrackDistance <= 0;
        }

        public (double requiredTrueCourse, double crossTrackError, double alongTrackDistance, double turnRadius) GetCourseInterceptInfo(SimAircraft aircraft)
        {
            // Otherwise calculate cross track error for this leg
            double crossTrackError = GeoUtil.CalculateCrossTrackErrorM(aircraft.Position.PositionGeoPoint, _endPoint.Point.PointPosition, _finalBearing,
                out double requiredTrueCourse, out double alongTrackDistance);

            if (alongTrackDistance <= AutopilotUtil.MIN_XTK_M && AutopilotUtil.MIN_XTK_M <= _prevAlongTrackDist)
            {
                aircraft.Fms.WaypointPassed?.Invoke(this, new WaypointPassedEventArgs(_endPoint.Point));
            }

            _prevAlongTrackDist = alongTrackDistance;

            return (requiredTrueCourse, crossTrackError, alongTrackDistance, -1);
        }

        public bool ShouldActivateLeg(SimAircraft aircraft, int intervalMs)
        {
            (double requiredTrueCourse, double crossTrackError, _, _) = GetCourseInterceptInfo(aircraft);

            // If there's no error
            double trackDelta = GeoUtil.CalculateTurnAmount(requiredTrueCourse, aircraft.Position.Track_True);
            if (Math.Abs(trackDelta) < double.Epsilon)
            {
                return false;
            }

            // Find cross track error to start turn (distance from intersection)
            double demandedTrack = AutopilotUtil.CalculateDemandedTrackOnCurrentTrack(crossTrackError, aircraft.Position.Track_True, requiredTrueCourse, aircraft.Position.Bank,
                aircraft.Position.GroundSpeed, intervalMs).demandedTrack;

            double requestedTurnDelta = GeoUtil.CalculateTurnAmount(demandedTrack, aircraft.Position.Track_True);
            return (trackDelta > 0 && requestedTurnDelta > 0 || trackDelta < 0 && requestedTurnDelta < 0);
        }

        public override string ToString()
        {
            return $"{_startPoint} =(TF)=> {_endPoint}";
        }

        public void ProcessLeg(SimAircraft aircraft, int intervalMs)
        {
        }

        public List<(GeoPoint start, GeoPoint end)> UiLines
        {
            get
            {
                var retList = new List<(GeoPoint start, GeoPoint end)>();
                retList.Add((StartPoint.Point.PointPosition, EndPoint.Point.PointPosition));
                return retList;
            }
        }
    }
}