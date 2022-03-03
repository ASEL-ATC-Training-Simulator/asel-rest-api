using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VatsimAtcTrainingSimulator.Core.Data
{
    public class Localizer : WaypointNavaid
    {
        private double _course;
        private GlidePath _glidepath;
        public Localizer(string identifier, double lat, double lon, string name, decimal frequency, double course, GlidePath glidepath = null) : base(identifier, lat, lon, name, frequency, NavaidType.LOC)
        {
            this._course = course;
            this._glidepath = glidepath;
        }

        public double Course => _course;
        public GlidePath Glidepath => _glidepath;
    }
}
