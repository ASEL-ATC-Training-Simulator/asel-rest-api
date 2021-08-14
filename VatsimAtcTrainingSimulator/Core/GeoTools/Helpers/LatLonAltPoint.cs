﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VatsimAtcTrainingSimulator.Core.GeoTools.Helpers
{
    /// <summary>
    /// A Class to contain a point on Earth
    /// </summary>
    public class LatLonAltPoint
    {
        private double _lat;
        private double _lon;
        private double _alt;

        public LatLonAltPoint(double lat, double lon, double alt)
        {
            Lat = lat;
            Lon = lon;
            _alt = alt;
        }

        public LatLonAltPoint(double lat, double lon) : this(lat, lon, 0) { }

        /// <summary>
        /// Latitude (degrees). -90 to +90.
        /// </summary>
        public double Lat
        {
            get => _lat;
            set => _lat = Math.Min(Math.Max(value, -90), 90);
        }

        /// <summary>
        /// Longitude (degrees). -180 to +180.
        /// </summary>
        public double Lon
        {
            get => _lon;
            set => _lon = AcftGeoUtil.NormalizeLongitude(value);
        }

        /// <summary>
        /// Altitude (feet).
        /// </summary>
        public double Alt
        {
            get => _alt;
            set => _alt = value;
        }

        /// <summary>
        /// Move this point on a bearing by a distance.
        /// </summary>
        /// <param name="bearing">Bearing to move point on (degrees). 0 to 360.</param>
        /// <param name="distance">Distance (meters) to move point by.</param>
        public void MoveByM(double bearing, double distance)
        {
            double R = AcftGeoUtil.EARTH_RADIUS_M + (_alt * AcftGeoUtil.CONV_FACTOR_M_FT);
            double bearingRads = AcftGeoUtil.DegreesToRadians(AcftGeoUtil.NormalizeHeading(bearing));
            double lat1 = AcftGeoUtil.DegreesToRadians(_lat);
            double lon1 = AcftGeoUtil.DegreesToRadians(_lon);

            double lat2 = Math.Asin(Math.Sin(lat1) * Math.Cos(distance / R) +
                Math.Cos(lat1) * Math.Sin(distance / R) * Math.Cos(bearingRads));
            double lon2 = lon1 + Math.Atan2(Math.Sin(bearingRads) * Math.Sin(distance / R) * Math.Cos(lat1),
                Math.Cos(distance / R) - Math.Sin(lat1) * Math.Sin(lat2));

            Lat = AcftGeoUtil.RadiansToDegrees(lat2);
            Lon = AcftGeoUtil.RadiansToDegrees(lon2);
        }

        /// <summary>
        /// Move this point on a bearing by a distance.
        /// </summary>
        /// <param name="bearing">Bearing to move point on (degrees). 0 to 360.</param>
        /// <param name="distance">Distance (meters) to move point by.</param>
        public void MoveByNMi(double bearing, double distance)
        {
            MoveByM(bearing, distance * AcftGeoUtil.CONV_FACTOR_NMI_M);
        }

        /// <summary>
        /// Calculates the intersection between 2 points and their bearings.
        /// </summary>
        /// <param name="point1">First Point</param>
        /// <param name="bearing1">Bearing from first point (degrees)</param>
        /// <param name="point2">Second Point</param>
        /// <param name="bearing2">Bearing from second point (degrees)</param>
        /// <returns><c>LatLonAltPoint</c> intersection or <c>null</c> if one does not exist.</returns>
        public static LatLonAltPoint Intersection(LatLonAltPoint point1, double bearing1, LatLonAltPoint point2, double bearing2)
        {
            // Conversions to radians
            double phi1 = AcftGeoUtil.DegreesToRadians(point1.Lat);
            double phi2 = AcftGeoUtil.DegreesToRadians(point2.Lat);
            double lambda1 = AcftGeoUtil.DegreesToRadians(point1.Lon);
            double lambda2 = AcftGeoUtil.DegreesToRadians(point2.Lon);
            double theta13 = AcftGeoUtil.DegreesToRadians(bearing1);
            double theta23 = AcftGeoUtil.DegreesToRadians(bearing2);
            double deltaPhi = phi2 - phi1;
            double deltaLambda = lambda2 - lambda1;

            // Angular distance (lat1, lon1) - (lat2, lon2)
            double sigma12 = 2 * Math.Asin(Math.Sqrt(Math.Pow(Math.Sin(deltaPhi / 2), 2)
                + Math.Cos(phi1) * Math.Cos(phi2) * Math.Pow(Math.Sin(deltaLambda / 2), 2)));

            // Coincident points
            if (sigma12 < Double.Epsilon)
            {
                return point1;
            }

            // Initial/Final Bearing between points
            double cosThetaA = (Math.Sin(phi2) - Math.Sin(phi1) * Math.Cos(sigma12)) / (Math.Sin(sigma12) * Math.Cos(phi1));
            double cosThetaB = (Math.Sin(phi1) - Math.Sin(phi2) * Math.Cos(sigma12)) / (Math.Sin(sigma12) * Math.Cos(phi2));
            double thetaA = Math.Acos(Math.Min(Math.Max(cosThetaA, -1), 1)); // Prevent rounding errors
            double thetaB = Math.Acos(Math.Min(Math.Max(cosThetaB, -1), 1)); // Prevent rounding errors

            double theta12 = Math.Sin(deltaLambda) > 0 ? thetaA : 2 * Math.PI - thetaA;
            double theta21 = Math.Sin(deltaLambda) > 0 ? 2 * Math.PI - thetaB : thetaB;

            double alpha1 = theta13 - theta12;// Angle 2-1-3
            double alpha2 = theta21 - theta23;// Angle 1-2-3

            // Infinite intersections
            if (Math.Sin(alpha1) == 0 && Math.Sin(alpha2) == 0)
            {
                return null;
            }

            // Ambiguous Intersection (antipodal?)
            if (Math.Sin(alpha1) * Math.Sin(alpha2) < 0)
            {
                return null;
            }

            double cosAlpha3 = -Math.Cos(alpha1) * Math.Cos(alpha2) + Math.Sin(alpha1) * Math.Sin(alpha2) * Math.Cos(sigma12);

            double sigma13 = Math.Atan2(Math.Sin(sigma12) * Math.Sin(alpha1) * Math.Sin(alpha2), Math.Cos(alpha2) + Math.Cos(alpha1) * cosAlpha3);

            double phi3 = Math.Asin(Math.Min(Math.Max(Math.Sin(phi1) * Math.Cos(sigma13) + Math.Cos(phi1) * Math.Sin(sigma13) * Math.Cos(theta13), -1), 1));

            double deltaLambda13 = Math.Atan2(Math.Sin(theta13) * Math.Sin(sigma13) * Math.Cos(phi1), Math.Cos(sigma13) - Math.Sin(phi1) * Math.Sin(phi3));

            double lambda3 = lambda1 + deltaLambda13;

            return new LatLonAltPoint(AcftGeoUtil.RadiansToDegrees(phi3), AcftGeoUtil.RadiansToDegrees(lambda3));
        }

        // override object.Equals
        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }

            LatLonAltPoint o = (LatLonAltPoint)obj;
            return _lat == o.Lat && _lon == o.Lon;
        }

        // override object.GetHashCode
        public override int GetHashCode()
        {
            int hashCode = _lat.GetHashCode() & 65535;
            hashCode <<= 16;
            hashCode += _lon.GetHashCode() & 65535;
            return hashCode;
        }
    }
}
