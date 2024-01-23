﻿using AviationCalcUtilNet.Aviation;
using AviationCalcUtilNet.Geo;
using AviationCalcUtilNet.Magnetic;
using NavData_Interface.Objects.Fixes;
using NavData_Interface.Objects.LegCollections.Legs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SaunaSim.Core.Simulator.Aircraft.FMS.Legs
{
    internal static class LegFactory
    {
        internal static IList<IRouteLeg> RouteLegsFromNavDataLegs(IList<Leg> leg, MagneticTileManager mcManager)
        {
            List<IRouteLeg> routeLegs = new List<IRouteLeg>();

            IEnumerator<Leg> enumerator = leg.GetEnumerator();

            Leg previousLeg = null;

            while (enumerator.MoveNext())
            {
                Leg currentLeg = enumerator.Current;

                switch (currentLeg.Type)
                {
                    case LegType.INITIAL_FIX:
                        // Ignore leg. Will be useful only for the next leg to process.
                        break;
                    case LegType.TRACK_TO_FIX:
                        // We SHOULD have a previous leg.
                        {
                            FmsPoint point1 = FmsPointFromNavDataLeg(previousLeg);
                            FmsPoint point2 = FmsPointFromNavDataLeg(currentLeg);

                            routeLegs.Add(new TrackToFixLeg(point1, point2));
                            break;
                        }
                    case LegType.COURSE_TO_FIX:
                        {
                            FmsPoint endPoint = FmsPointFromNavDataLeg(currentLeg);

                            routeLegs.Add(new CourseToFixLeg(endPoint, BearingTypeEnum.MAGNETIC, currentLeg.OutboundMagneticCourse, mcManager));
                            break;
                        }
                    case LegType.DIRECT_TO_FIX:
                        {
                            FmsPoint directPoint = FmsPointFromNavDataLeg(currentLeg);

                            routeLegs.Add(new DirectToFixLeg(directPoint)); // compile error - awaiting DF refactor
                            break;
                        }
                    case LegType.FIX_TO_ALT:
                        {
                            FmsPoint startPoint = FmsPointFromNavDataLeg(currentLeg);
                            routeLegs.Add(new FixToAltLeg(startPoint, BearingTypeEnum.MAGNETIC, currentLeg.OutboundMagneticCourse, currentLeg.LowerAltitudeConstraint, mcManager));
                            break;
                        }
                    case LegType.TRACK_TO_DISTANCE:
                        {
                            // TODO: make a track to distance leg

                            break;
                        }
                    case LegType.TRACK_TO_DME:
                        {
                            // TODO: make a track to DME leg
                            break;
                        }
                    case LegType.FIX_TO_MANUAL:
                        {
                            FmsPoint startPoint = FmsPointFromNavDataLeg(currentLeg);
                            routeLegs.Add(new FixToManualLeg(startPoint, BearingTypeEnum.MAGNETIC, currentLeg.OutboundMagneticCourse, mcManager));
                            break;
                        }
                    case LegType.COURSE_TO_ALT:
                        {
                            routeLegs.Add(new CourseToAltLeg(currentLeg.LowerAltitudeConstraint, BearingTypeEnum.MAGNETIC, currentLeg.OutboundMagneticCourse, mcManager));
                            break;
                        }
                    case LegType.COURSE_TO_DME:
                        {
                            // TODO: make a course to DME leg
                            break;
                        }
                    case LegType.COURSE_TO_INTC:
                        {
                            // TODO: make a course to intercept leg
                            break;
                        }
                    case LegType.COURSE_TO_RADIAL:
                        {
                            // TODO: make a course to radial leg
                            break;
                        }
                    case LegType.RADIUS_TO_FIX:
                        {
                            FmsPoint startPoint = FmsPointFromNavDataLeg(previousLeg);
                            FmsPoint endPoint = FmsPointFromNavDataLeg(currentLeg);
                            Fix centerPoint = currentLeg.CenterPoint;

                            Bearing initialTrueCourse = routeLegs.Last().FinalTrueCourse;
                            Bearing finalTrueCourse = mcManager.MagneticToTrue(startPoint.Point.PointPosition, DateTime.Now, currentLeg.OutboundMagneticCourse);

                            routeLegs.Add(new RadiusToFixLeg(startPoint, endPoint, centerPoint, initialTrueCourse, finalTrueCourse));

                            break;
                        }
                    case LegType.ARC_TO_FIX:
                        {
                            // TODO: make an arc to fix leg
                            break;
                        }
                    case LegType.HEADING_TO_ALT:
                        {
                            // TODO: heading to alt leg
                            break;
                        }
                    case LegType.HEADING_TO_DME:
                        {
                            // TODO: heading legs
                            break;
                        }
                    case LegType.HEADING_TO_INTC:
                        {
                            // TODO: heading legs
                            break;
                        }
                    case LegType.HEADING_TO_MANUAL:
                        {
                            // TODO: heading legs
                            break;
                        }
                    case LegType.HEADING_TO_RADIAL:
                        {
                            // TODO: heading legs
                            break;
                        }
                    case LegType.PROCEDURE_TURN:
                        {
                            // TODO: procedure turns
                            break;
                        }
                    case LegType.HOLD_TO_ALT:
                        {
                            // TODO: hold to altitude
                            break;
                        }
                    case LegType.HOLD_TO_FIX:
                        {
                            // TODO: hold to fix
                            break;
                        }
                    case LegType.HOLD_TO_MANUAL:
                        {
                            new HoldToManualLeg()
                            break;
                        }
                }

                previousLeg = currentLeg;
            }
        }

        internal static FmsPoint FmsPointFromNavDataLeg(Leg leg)
        {
            FmsPoint point = new FmsPoint(new RouteWaypoint(leg.EndPoint),
                leg.EndPointDescription.IsFlyOver ? RoutePointTypeEnum.FLY_OVER : RoutePointTypeEnum.FLY_BY
                );

            point.LowerAltitudeConstraint = leg.LowerAltitudeConstraint;
            point.UpperAltitudeConstraint = leg.UpperAltitudeConstraint;

            if (leg.SpeedType != null)
            {
                point.SpeedConstraint = leg.SpeedRestriction;
                switch (leg.SpeedType)
                {
                    case SpeedRestrictionType.BELOW:
                        point.SpeedConstraintType = ConstraintType.LESS;
                        break;
                    case SpeedRestrictionType.AT:
                        point.SpeedConstraintType = ConstraintType.EXACT;
                        break;
                    case SpeedRestrictionType.ABOVE:
                        point.SpeedConstraintType = ConstraintType.MORE;
                        break;
                    default:
                        point.SpeedConstraintType = ConstraintType.FREE;
                        break;
                }
            }

            return point;
        }

    }
}
