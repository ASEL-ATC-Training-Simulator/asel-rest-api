﻿using AviationCalcUtilNet.GeoTools;
using NavData_Interface.Objects;
using NavData_Interface.Objects.Fixes;
using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;

namespace NavData_Interface.DataSources
{
    public abstract class DataSource
    {
        public abstract string GetId();

        public abstract List<Fix> GetFixesByIdentifier(string identifier);

        public abstract Localizer GetLocalizerFromAirportRunway(string airportIdentifier, string runwayIdentifier);

        public abstract Airport GetClosestAirportWithinRadius(GeoPoint position, double radiusM);

        public abstract Runway GetRunwayFromAirportRunwayIdentifier(string airportIdentifier, string runwayIdentifier);

        public Fix GetClosestFixByIdentifier(GeoPoint point, string identifier)
        {
            var fixes = GetFixesByIdentifier(identifier);
            Fix closestFix = null;
            double closestDistance = double.MaxValue;

            foreach (var fix in fixes)
            {
                double distance = GeoPoint.Distance(point, fix.Location).Meters;
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestFix = fix;
                }
            }

            return closestFix;
        }

        public override bool Equals(object obj)
        {
            if (obj == this)
            {
                return true;
            }

            if (obj is DataSource)
            {
                var otherSource = (DataSource)obj;

                return otherSource.GetId() == this.GetId();
            }

            return false;
        }

        public override int GetHashCode()
        {
            return GetId().GetHashCode();
        }
    }
}
