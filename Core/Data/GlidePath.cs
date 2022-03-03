namespace VatsimAtcTrainingSimulator.Core.Data
{
    public class GlidePath
    {
        private double _lat;
        private double _lon;
        private double _angle;
        private double _alt;

        public GlidePath(double lat, double lon, double alt = 0, double angle = 3.0)
        {
            this._lat = lat;
            this._lon = lon;
            this._alt = alt;
            this._angle = angle;
        }

        public double Lat => _lat;
        public double Lon => _lon;
        public double Angle => _angle;
        public double Alt => _alt;
    }
}
