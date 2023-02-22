using AviationCalcUtilNet.GeoTools;
using AviationCalcUtilNet.GeoTools.GribTools;
using AviationCalcUtilNet.GeoTools.MagneticTools;
using AviationCalcUtilNet.MathTools;
using System;
using SaunaSim.Core.Simulator.Aircraft.Performance;
using System.Collections.Generic;
using System.Numerics;
using SaunaSim.Core.Data;
using FsdConnectorNet;

namespace SaunaSim.Core.Simulator.Aircraft
{
    public class AircraftPosition
    {
        private double _altInd;
        private double _altPres;
        private double _altDens;
        private double _altTrue;
        private Vector3 _airVelocity;
        private Vector3 _groundVelocity;
        private double _magneticHdg;
        private double _altSetting_hPa = AtmosUtil.ISA_STD_PRES_hPa;
        private double _sfcPress_hPa = AtmosUtil.ISA_STD_PRES_hPa;
        private double _ias;
        private double _mach;
        private GribDataPoint _gribPoint;

        // Position
        public double Latitude { get; set; }
        public double Longitude { get; set; }

        public double IndicatedAltitude
        {
            get => _altInd;
            set
            {
                _altInd = value;
                _altPres = AtmosUtil.ConvertIndicatedToPressureAlt(_altInd, _altSetting_hPa);
                _altTrue = AtmosUtil.ConvertIndicatedToAbsoluteAlt(_altInd, _altSetting_hPa, SurfacePressure_hPa);
                if (_gribPoint != null)
                {
                    double T = AtmosUtil.CalculateTempAtAlt(MathUtil.ConvertFeetToMeters(_altTrue), _gribPoint.GeoPotentialHeight_M, _gribPoint.Temp_K);
                    double p = AtmosUtil.CalculatePressureAtAlt(MathUtil.ConvertFeetToMeters(_altTrue), _gribPoint.GeoPotentialHeight_M, _gribPoint.Level_hPa * 100, T);
                    _altDens = MathUtil.ConvertMetersToFeet(AtmosUtil.CalculateDensityAltitude(p, T));
                }
                else
                {
                    double T = MathUtil.ConvertCelsiusToKelvin(AtmosUtil.CalculateIsaTemp(_altPres));
                    double p = AtmosUtil.ISA_STD_PRES_Pa;
                    _altDens = MathUtil.ConvertMetersToFeet(AtmosUtil.CalculateDensityAltitude(p, T));
                }
            }
        }

        public double AbsoluteAltitude
        {
            get => _altTrue;
            set
            {
                _altTrue = value;
                _altInd = AtmosUtil.ConvertAbsoluteToIndicatedAlt(_altTrue, _altSetting_hPa, _sfcPress_hPa);
                _altPres = AtmosUtil.ConvertIndicatedToPressureAlt(_altInd, _altSetting_hPa);
                if (_gribPoint != null)
                {
                    double T = AtmosUtil.CalculateTempAtAlt(MathUtil.ConvertFeetToMeters(_altTrue), _gribPoint.GeoPotentialHeight_M, _gribPoint.Temp_K);
                    double p = AtmosUtil.CalculatePressureAtAlt(MathUtil.ConvertFeetToMeters(_altTrue), _gribPoint.GeoPotentialHeight_M, _gribPoint.Level_hPa * 100, T);
                    _altDens = MathUtil.ConvertMetersToFeet(AtmosUtil.CalculateDensityAltitude(p, T));
                }
                else
                {
                    double T = MathUtil.ConvertCelsiusToKelvin(AtmosUtil.CalculateIsaTemp(_altPres));
                    double p = AtmosUtil.ISA_STD_PRES_Pa;
                    _altDens = MathUtil.ConvertMetersToFeet(AtmosUtil.CalculateDensityAltitude(p, T));
                }
            }
        }

        public double PressureAltitude => _altPres;
        public double DensityAltitude => _altDens;

        // Rotation
        public double Heading_Mag
        {
            get => _magneticHdg;

            set
            {
                _magneticHdg = value;

                // Calculate True Heading
                double newTrueHdg = MathUtil.ConvertDegreesToRadians(MagneticUtil.ConvertMagneticToTrueTile(_magneticHdg, PositionGeoPoint));
                double latTas = Math.Sqrt(Math.Pow(_airVelocity.X, 2) + Math.Pow(_airVelocity.Z, 2));
                if (latTas <= 0)
                {
                    latTas = 1;
                }
                _airVelocity.X = (float)(latTas * Math.Sin(newTrueHdg));
                _airVelocity.Z = (float)(latTas * Math.Cos(newTrueHdg));

                // Calculate True Track
                _groundVelocity = _airVelocity - Wind;
            }
        }

        public double Heading_True
        {
            get => MathUtil.ConvertRadiansToDegrees(Math.Atan2(_airVelocity.X, _airVelocity.Z));
            set
            {
                // Calculate True Heading
                double newTrueHdg = MathUtil.ConvertDegreesToRadians(value);
                double latTas = Math.Sqrt(Math.Pow(_airVelocity.X, 2) + Math.Pow(_airVelocity.Z, 2));
                if (latTas <= 0)
                {
                    latTas = 1;
                }
                _airVelocity.X = (float)(latTas * Math.Sin(newTrueHdg));
                _airVelocity.Z = (float)(latTas * Math.Cos(newTrueHdg));

                // Set Magnetic Heading
                _magneticHdg = MagneticUtil.ConvertTrueToMagneticTile(value, PositionGeoPoint);

                // Calculate True Track
                _groundVelocity = _airVelocity - Wind;
            }
        }

        public double Track_True
        {
            get => MathUtil.ConvertRadiansToDegrees(Math.Atan2(_groundVelocity.X, _groundVelocity.Z));
            set
            {
                // Calculate True Track
                double newTrueTrack = MathUtil.ConvertDegreesToRadians(value);
                double latGs = Math.Sqrt(Math.Pow(_groundVelocity.X, 2) + Math.Pow(_groundVelocity.Z, 2));
                if (latGs <= 0)
                {
                    latGs = 1;
                }
                _groundVelocity.X = (float)(latGs * Math.Sin(newTrueTrack));
                _groundVelocity.Z = (float)(latGs * Math.Cos(newTrueTrack));

                // Calculate True Heading
                _groundVelocity = _airVelocity + Wind;

                // Set Magnetic Heading
                _magneticHdg = MagneticUtil.ConvertTrueToMagneticTile(MathUtil.ConvertRadiansToDegrees(Math.Atan2(_airVelocity.X, _airVelocity.Z)), PositionGeoPoint);
            }
        }

        public double Fpa =>
            MathUtil.ConvertRadiansToDegrees(
                Math.Atan2(
                    _groundVelocity.Y,
                    Math.Sqrt(Math.Pow(_groundVelocity.X, 2) + Math.Pow(_groundVelocity.Z, 2))
                )
            );

        public double Bank { get; set; }
        public double Pitch { get; set; }

        // Linear Velocities
        public Vector3 AirVelocity
        {
            get => _airVelocity;
            set
            {
                // True Airspeed
                _airVelocity = value;
                // Ground Speed
                _groundVelocity = value - Wind;
                // Indicated Airspeed
                _ias = AtmosUtil.ConvertTasToIas(
                    MathUtil.ConvertMpersToKts(_airVelocity.Length()),
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _mach
                );
            }
        }

        public Vector3 GroundVelocity
        {
            get => _groundVelocity;
            set
            {
                // Ground Speed
                _groundVelocity = value;
                // True Airspeed
                _airVelocity = value + Wind;
                // Indicated Airspeed
                _ias = AtmosUtil.ConvertTasToIas(
                    MathUtil.ConvertMpersToKts(_airVelocity.Length()),
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _mach
                );
            }
        }

        public double IndicatedAirSpeed
        {
            get => _ias;
            set
            {
                // Indicated Airspeed
                _ias = value;
                // True Airspeed
                double newTas = AtmosUtil.ConvertIasToTas(
                    _ias,
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _mach
                );
                _airVelocity = Vector3.Normalize(_airVelocity) * (float)MathUtil.ConvertKtsToMpers(newTas);
                // Ground Speed
                _groundVelocity = _airVelocity - Wind;
            }
        }

        public double TrueAirSpeed
        {
            get => MathUtil.ConvertMpersToKts(_airVelocity.Length());
            set
            {
                // True Airspeed
                _airVelocity = Vector3.Normalize(_airVelocity) * (float)MathUtil.ConvertKtsToMpers(value);
                // Ground Speed
                _groundVelocity = _airVelocity - Wind;
                // Indicated Airspeed
                _ias = AtmosUtil.ConvertTasToIas(
                    MathUtil.ConvertMpersToKts(_airVelocity.Length()),
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _mach
                );
            }
        }

        public double GroundSpeed
        {
            get => MathUtil.ConvertMpersToKts(_groundVelocity.Length());
            set
            {
                // Ground Speed
                _groundVelocity = Vector3.Normalize(_groundVelocity) * (float)MathUtil.ConvertKtsToMpers(value);
                // True Airspeed
                _airVelocity = _groundVelocity + Wind;
                // Indicated Airspeed
                _ias = AtmosUtil.ConvertTasToIas(
                    MathUtil.ConvertMpersToKts(_airVelocity.Length()),
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _mach
                );
            }
        }

        public double MachNumber
        {
            get => _mach;
            set
            {
                // Mach #
                _mach = value;
                // True Airspeed
                double T = AtmosUtil.CalculateTempAtAlt(
                    MathUtil.ConvertFeetToMeters(_altTrue),
                    _gribPoint?.GeoPotentialHeight_M ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K
                );
                double newTas = AtmosUtil.ConvertMachToTas(_mach, T);
                _airVelocity = Vector3.Normalize(_airVelocity) * (float)newTas;
                // Indicated Airspeed
                _ias = AtmosUtil.ConvertTasToIas(
                    MathUtil.ConvertMpersToKts(_airVelocity.Length()),
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _
                );
                // Ground Speed
                _groundVelocity = _airVelocity - Wind;
            }
        }

        public double VerticalSpeed
        {
            get => PerfDataHandler.ConvertMpersToFpm(_groundVelocity.Y);
            set => _groundVelocity.Y = (float)PerfDataHandler.ConvertFpmToMpers(value);
        }

        public double WindDirection => _gribPoint?.WDir_deg ?? 0;
        public double WindSpeed => _gribPoint?.WSpeed_kts ?? 0;

        // Rotational Velocities
        public double Heading_Velocity_RadPerS => 0;
        public double Bank_Velocity_RadPerS => 0;
        public double Pitch_Velocity_RadPerS => 0;

        // Atmospheric Data        
        public double AltimeterSetting_hPa
        {
            get => _altSetting_hPa;
            set
            {
                _altSetting_hPa = value;

                // Backwards compute new Indicated Alt
                _altInd = AtmosUtil.ConvertAbsoluteToIndicatedAlt(_altTrue, _altSetting_hPa, _sfcPress_hPa);
                _altPres = AtmosUtil.ConvertIndicatedToPressureAlt(_altInd, _altSetting_hPa);
            }
        }

        public double SurfacePressure_hPa
        {
            get => _sfcPress_hPa;
            set
            {
                _sfcPress_hPa = value;

                // Backwards compute new Indicated Alt
                _altInd = AtmosUtil.ConvertAbsoluteToIndicatedAlt(_altTrue, _altSetting_hPa, _sfcPress_hPa);
                _altPres = AtmosUtil.ConvertIndicatedToPressureAlt(_altInd, _altSetting_hPa);
            }
        }

        public double WindXComp => WindSpeed * Math.Sin((Heading_True - WindDirection) * Math.PI / 180.0);

        public double WindHComp => GeoUtil.HeadwindComponent(WindSpeed, WindDirection, Heading_True);

        public Vector3 Wind => new Vector3((float)-(_gribPoint?.U_mpers ?? 0), 0, (float)-(_gribPoint?.V_mpers ?? 0));

        public GribDataPoint GribPoint
        {
            get => _gribPoint;
            set
            {
                _gribPoint = value;

                SurfacePressure_hPa = _gribPoint != null && _gribPoint.SfcPress_hPa != 0 ? _gribPoint.SfcPress_hPa : AtmosUtil.ISA_STD_PRES_hPa;

                // Calculate Indicated Airspeed
                _ias = AtmosUtil.ConvertTasToIas(
                    MathUtil.ConvertMpersToKts(_airVelocity.Length()),
                    _gribPoint?.Level_hPa ?? AtmosUtil.ISA_STD_PRES_hPa,
                    _altTrue,
                    _gribPoint?.GeoPotentialHeight_Ft ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K,
                    out _mach
                );

                // Calculate Ground speed
                _groundVelocity = _airVelocity - Wind;

                // Density Alt
                double T = AtmosUtil.CalculateTempAtAlt(
                    MathUtil.ConvertFeetToMeters(_altTrue),
                    _gribPoint?.GeoPotentialHeight_M ?? 0,
                    _gribPoint?.Temp_K ?? AtmosUtil.ISA_STD_TEMP_K
                );
                double p = AtmosUtil.CalculatePressureAtAlt(
                    MathUtil.ConvertFeetToMeters(_altTrue),
                    _gribPoint?.GeoPotentialHeight_M ?? 0,
                    _gribPoint?.Level_hPa * 100 ?? AtmosUtil.ISA_STD_PRES_Pa,
                    T
                );
                _altDens = MathUtil.ConvertMetersToFeet(AtmosUtil.CalculateDensityAltitude(p, T));
            }
        }

        public GeoPoint PositionGeoPoint => new GeoPoint(Latitude, Longitude, AbsoluteAltitude);

        public void UpdateGribPoint()
        {
            GribTile tile = GribTile.FindOrCreateGribTile(PositionGeoPoint, DateTime.UtcNow);

            GribPoint = tile?.GetClosestPoint(PositionGeoPoint);
        }
    }
}