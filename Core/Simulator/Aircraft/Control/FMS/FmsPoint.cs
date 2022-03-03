using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VatsimAtcTrainingSimulator.Core.Data;

namespace VatsimAtcTrainingSimulator.Core.Simulator.Aircraft.Control.FMS
{
    public enum RoutePointTypeEnum
    {
        FLY_BY,
        FLY_OVER
    }

    public class FmsPoint
    {
        private IRoutePoint _point;
        private RoutePointTypeEnum _routePointType;

        public FmsPoint(IRoutePoint point, RoutePointTypeEnum type)
        {
            _point = point;
            _routePointType = type;
        }

        public IRoutePoint Point => _point;

        public RoutePointTypeEnum PointType { get => _routePointType; set => _routePointType = value; }

        public double LowerAltitudeConstraint { get; set; }

        public double UpperAltitudeConstraint { get; set; }

        public ConstraintType SpeedConstraintType { get; set; } = ConstraintType.FREE;

        public double SpeedConstraint { get; set; }

        public override string ToString()
        {
            return _point.PointName;
        }
    }
}
