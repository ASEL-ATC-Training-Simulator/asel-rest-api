using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using AviationCalcUtilNet.GeoTools;
using AviationCalcUtilNet.MathTools;

namespace SaunaSim.Core.Simulator.Aircraft.Performance
{
    public struct LiftDragDataPoint
    {
        public double mass_kg;
        public double ias_mpers;
        public double thrustLever;
        public double altitude_m;
        public double pitch_rads;
        public double fpa_rads;
    }
    public static class PerfDataHandler
    {
        public static void Test1000()
        {
            List<LiftDragDataPoint> dataPoints = new List<LiftDragDataPoint>()
            {
                new LiftDragDataPoint()
                {
                    mass_kg = 80000 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(180),
                    thrustLever = 56 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(3000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(4.8),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 79990 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(210),
                    thrustLever = 58 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(3000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(3),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 79700 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(250),
                    thrustLever = 63.4 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(3000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(2),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 79500 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(250),
                    thrustLever = 92.4 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(6000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(8),
                    fpa_rads = MathUtil.ConvertDegreesToRadians(7.5)
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 79200 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(250),
                    thrustLever = 68 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(10000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(2),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 78200 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(250),
                    thrustLever = 95 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(26000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(3),
                    fpa_rads = MathUtil.ConvertDegreesToRadians(2.5)
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 77700 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(250),
                    thrustLever = 86 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(30000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(2),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 77500 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(300),
                    thrustLever = 88 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(30000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(0.1),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 77200 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(197),
                    thrustLever = 74 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(30000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(3),
                    fpa_rads = 0
                },
                new LiftDragDataPoint()
                {
                    mass_kg = 77000 / 2.205,
                    ias_mpers = MathUtil.ConvertKtsToMpers(146),
                    thrustLever = 82 - 22.2,
                    altitude_m = MathUtil.ConvertFeetToMeters(30000),
                    pitch_rads = MathUtil.ConvertDegreesToRadians(7.5),
                    fpa_rads = 0
                },
            };

            foreach (var dp in dataPoints)
            {
                // Convert Inputs
                double h = dp.altitude_m;
                double h0 = 0;
                double T0 = AtmosUtil.ISA_STD_TEMP_K;
                double p0 = AtmosUtil.ISA_STD_PRES_Pa;
                double V = AtmosUtil.ConvertIasToTas(MathUtil.ConvertMpersToKts(dp.ias_mpers), AtmosUtil.ISA_STD_PRES_hPa, MathUtil.ConvertMetersToFeet(dp.altitude_m), 0,
                    AtmosUtil.ISA_STD_TEMP_K, out double mach);
                double pitch = dp.pitch_rads;
                double fpa = dp.fpa_rads;
                double nose_bearing = 0;
                double thrustLever = dp.thrustLever / 100.0;
                double m = dp.mass_kg;
                double bank = 0;
                PerfData perfData = LookupForAircraft("E75L");

                // Calculate pressure, temp, and density at height
                double T = AtmosUtil.CalculateTempAtAlt(h, h0, T0);
                double p = AtmosUtil.CalculatePressureAtAlt(h, h0, p0, T);
                double rho = AtmosUtil.CalculateDryAirDensity(p, T);

                // Calculate Angle of Attack
                double alpha = pitch - fpa;

                Vector3 totalForce = new Vector3(0, 0, 0);

                // Calculate Thrust (Ft)
                double Ft = GetThrustAtAltitude(perfData, thrustLever, p, T);
                Vector3 thrust = CreateVector(Ft, pitch, nose_bearing);

                // Calculate Weight (Fg)
                double Fg = m * AtmosUtil.EARTH_G;
                Vector3 weight = new Vector3(0, (float) -Fg, 0);

                Vector3 remainingForce = totalForce - thrust - weight;
                Vector3 dragUnitVector = CreateUnitVector(fpa, nose_bearing);
                Vector3 liftUnitVector = CreateUnitVector(pitch + 0.5 * Math.PI, nose_bearing + bank);
                Vector3 drag = Vector3.Dot(remainingForce, dragUnitVector) * dragUnitVector;
                Vector3 lift = Vector3.Dot(remainingForce, liftUnitVector) * liftUnitVector;

                double Fd = drag.Length();
                double Fl = lift.Length();
                
                // 0.5 * rho * Math.Pow(tas_mpers, 2) * s_sqm * cl;
                double cl = Fl / (0.5 * rho * Math.Pow(V, 2) * perfData.WingArea_sqm);
                double cd = Fd / (0.5 * rho * Math.Pow(V, 2) * perfData.WingArea_sqm);

                double testcl = CalculateLiftCoefficient(alpha);
                double testFl = CalculateLiftForce(rho, V, perfData.WingArea_sqm, testcl);

                double testcd = CalculateDragCoefficient(alpha);
                double testFd = CalculateDragForce(rho, V, perfData.WingArea_sqm, testcd);

                Console.WriteLine();
                Console.WriteLine($"alpha\t{alpha}");
                Console.WriteLine($"tas\t{MathUtil.ConvertMpersToKts(V)}");
                Console.WriteLine($"Drag\t{(int) Fd}\t{(int) testFd}");
                Console.WriteLine($"Lift\t{(int) Fl}\t{(int) testFl}");
                Console.WriteLine($"cd\t{cd}");
                Console.WriteLine($"cl\t{cl}");
            }
        }
        
        
        
        public static PerfData LookupForAircraft(string icaoEquip)
        {
            PerfData e175 = new PerfData()
            {
                OEW_kg = 21886,
                MZFW_kg = 31700,
                MaxFuel_kg = 9335,
                MTOW_kg = 38790,
                MLW_kg = 34000,
                Climb_KIAS = 270,
                Climb_Mach = 0.73,
                Cruise_KIAS = 290,
                Cruise_Mach = 0.78,
                Descent_KIAS = 290,
                Descent_Mach = 0.78,
                MaxThrust_N = 64000,
                IdleThrust_N = 17800,
                Engines = 2,
                EngineType = EngineType.TURBOFAN,
                WingArea_sqm = 72.72,
                CrossSectionArea_sqm = 27.25,
                WingSpan_m = 28.65,
                ConfigList = new List<PerfData.ConfigSetting>()
                {
                    new PerfData.ConfigSetting()
                    {
                        GearDown = false,
                        MaxKias = 320,
                        MinKias = 180,
                        NormKias = 250
                    },
                    new PerfData.ConfigSetting()
                    {
                        GearDown = false,
                        MaxKias = 230,
                        MinKias = 160,
                        NormKias = 210
                    },
                    new PerfData.ConfigSetting()
                    {
                        GearDown = false,
                        MaxKias = 215,
                        MinKias = 150
                    },
                    new PerfData.ConfigSetting()
                    {
                        GearDown = true,
                        MaxKias = 200,
                        MinKias = 140,
                        NormKias = 160
                    },
                    new PerfData.ConfigSetting()
                    {
                        GearDown = true,
                        MaxKias = 180,
                        MinKias = 110,
                        NormKias = 140
                    }
                }
            };

            return e175;
        }

        // https://calculator.academy/oswald-efficiency-factor-calculator/#f1p1|f2p0
        public static double CalculateOswaldEfficiencyNumber(double wingspan, double wing_area)
        {
            double ar = Math.Pow(wingspan, 2) / wing_area;
            return 1.78 * (1 - .045 * Math.Pow(ar, .68)) - .64;
        }

        // https://aerospaceweb.org/question/airfoils/q0150b.shtml
        public static double CalculateLiftCoefficient(double alpha_rads)
        {
            if (alpha_rads < 0)
            {
                return 0;
            }

            var linReg = LinearRegression(0, 0.2, 0.07, 0.6);

            if (alpha_rads < 0.2)
            {
                return linReg(alpha_rads);
            }

            if (alpha_rads < 0.26)
            {
                return -223.81 * Math.Pow(alpha_rads, 2) + 93.119 * alpha_rads - 9.07143;
            }

            return 0;
        }

        /*public static Func<double, double> CreateBezierFunction((double, double) p1, (double, double) p2, (double, double) p3)
        {
            //x = (1−t)2x1 + 2(1−t)tx2 + t2x3
            //y = (1−t)2y1 + 2(1−t)ty2 + t2y3
            return (double x) =>
            {
                
            };
        }*/

        // https://aerospaceweb.org/question/airfoils/q0150b.shtml
        public static double CalculateDragCoefficient(double alpha_rads)
        {
            return 2.8 * Math.Pow(alpha_rads, 2) + 0.01;
        }

        // https://en.wikipedia.org/wiki/Lift_(force)
        public static double CalculateLiftForce(double rho, double tas_mpers, double s_sqm, double cl)
        {
            return 0.5 * rho * Math.Pow(tas_mpers, 2) * s_sqm * cl;
        }

        // https://en.wikipedia.org/wiki/Drag_(physics)
        public static double CalculateDragForce(double rho, double tas_mpers, double a_sqm, double cd)
        {
            return 0.5 * rho * Math.Pow(tas_mpers, 2) * cd * a_sqm;
        }

        /// <summary>
        /// Converts bearing and pitch to a unit vector.
        /// </summary>
        /// <param name="pitch_rads">Pitch relative to horizon (radians)</param>
        /// <param name="bearing_rads">True bearing relative to true north (radians)</param>
        /// <returns>Unit Vector</returns>
        public static Vector3 CreateUnitVector(double pitch_rads, double bearing_rads) => CreateVector(1, pitch_rads, bearing_rads);

        public static Vector3 CreateVector(double magnitude, double pitch_rads, double bearing_rads)
        {
            // X = Longitude
            // Y = Altitude
            // Z = Latitude
            double y = magnitude * Math.Sin(pitch_rads);
            double r = magnitude * Math.Cos(pitch_rads);
            double x = r * Math.Sin(bearing_rads);
            double z = r * Math.Cos(bearing_rads);
            
            return new Vector3((float) x, (float) y, (float) z);
        }

        public static double ConvertFpmToMpers(double fpm)
        {
            return MathUtil.ConvertFeetToMeters(fpm) / 60;
        }
        
        public static double ConvertMpersToFpm(double mpers)
        {
            return MathUtil.ConvertMetersToFeet(60 * mpers);
        }
        
        /// <summary>
        /// Calculates the net force acting on the aircraft.
        /// </summary>
        /// <param name="perfData">Performance data for the aircraft</param>
        /// <param name="airVelocity">Velocity vector with respect to the air mass (m/s)</param>
        /// <param name="groundVelocity">Velocity vector with respect to the ground (m/s)</param>
        /// <param name="pitch">Aircraft pitch (radians)</param>
        /// <param name="bank">Aircraft bank (radians)</param>
        /// <param name="m">Aircraft mass (kg)</param>
        /// <param name="thrustLever">Thrust Lever Position (%/100)</param>
        /// <param name="p0">Reference Pressure (Pa)</param>
        /// <param name="h">True Altitude (m)</param>
        /// <param name="h0">Reference Altitude (m)</param>
        /// <param name="T0">Reference Temperature (K)</param>
        /// <returns>Force Vector (N)</returns>
        public static Vector3 CalculateForces(PerfData perfData, Vector3 airVelocity, Vector3 groundVelocity, double pitch, double bank, double m,
            double thrustLever, double p0, double h, double h0, double T0)
        {
            // Convert Inputs
            double V = airVelocity.Length();
            double fpa = Math.Atan2(groundVelocity.Y, Math.Sqrt(Math.Pow(groundVelocity.X, 2) + Math.Pow(groundVelocity.Z, 2)));
            double nose_bearing = Math.Atan2(airVelocity.X, airVelocity.Z);

            // Calculate pressure, temp, and density at height
            double T = AtmosUtil.CalculateTempAtAlt(h, h0, T0);
            double p = AtmosUtil.CalculatePressureAtAlt(h, h0, p0, T);
            double rho = AtmosUtil.CalculateDryAirDensity(p, T);

            // Calculate Angle of Attack
            double alpha = pitch - fpa;

            // Calculate Thrust (Ft)
            double Ft = GetThrustAtAltitude(perfData, thrustLever, p, T);
            Vector3 thrust = CreateVector(Ft, pitch, nose_bearing);

            // Calculate Drag (Fd)
            double Cd = CalculateDragCoefficient(alpha);
            double Fd = CalculateDragForce(rho, V, perfData.WingArea_sqm, Cd);
            Vector3 drag = CreateVector(-Fd, fpa, nose_bearing);

            // Calculate Weight (Fg)
            double Fg = m * AtmosUtil.EARTH_G;
            Vector3 weight = new Vector3(0, (float) -Fg, 0);

            // Calculate Lift (Fl)
            double Cl = CalculateLiftCoefficient(alpha);
            double Fl = CalculateLiftForce(rho, V, perfData.WingArea_sqm, Cl);
            Vector3 lift = CreateVector(Fl, pitch + 0.5 * Math.PI, nose_bearing + bank);

             return thrust + lift + drag + weight;
        }

        public static Vector3 CalculateAccels(Vector3 force, double mass_kg)
        {
            return force  / (float) mass_kg;
            // Convert inputs
            /*double alpha = pitch_rads - fpa_rads;
            double theta = fpa_rads;
            double phi = pitch_rads;
            
            // Get Lift without bank involved
            double FlNoBank = Fl * Math.Cos(bank_rads);
            
            // Get Parallel forces to the airplane's FPA
            double FlParallel = FlNoBank * Math.Sin(-alpha);
            double FgParallel = Fg * Math.Sin(theta);
            double FtParallel = Ft * Math.Cos(alpha);
            double F_fpa = FtParallel + Fd + FgParallel + FlParallel;

            // Get Vertical forces
            double FlVertical = FlNoBank * Math.Cos(phi);
            double FtVertical = Ft * Math.Sin(phi);
            double FdVertical = Fd * Math.Sin(theta);
            double F_vert = FtVertical + FlVertical + Fg + FdVertical;

            // Return accelerations
            return (F_fpa / mass_kg, F_vert / mass_kg);*/
        }

        public static double CalculateFinalVelocity(double Vi, double a, double t)
        {
            return Vi + a * t;
        }

        
        public static Vector3 CalculateVelocities(Vector3 Vi, Vector3 a, double t_s)
        {
            return new Vector3(
                (float)CalculateFinalVelocity(Vi.X, a.X, t_s),
                (float)CalculateFinalVelocity(Vi.Y, a.Y, t_s),
                (float)CalculateFinalVelocity(Vi.Z, a.Z, t_s)
            );
            //return (CalculateFinalVelocity(Vi_fpa_mpers, a_fpa_mperssqd, t_s), CalculateFinalVelocity(Vi_vert_mpers, a_vert_mperssqd, t_s));
        }

        private static Func<double, double> LinearRegression(double x1, double x2, double y1, double y2)
        {
            double m = (y2 - y1) / (x2 - x1);
            double b = y1 - (m * x1);
            // 0.1 = m * 0 + b
            return (x) => m * x + b;
        }

        public static double GetThrustAtAltitude(PerfData perfData, double thrustLever, double p, double T)
        {
            double totalMaxThrust = perfData.MaxThrust_N * perfData.Engines;
            double totalIdleThrust = perfData.IdleThrust_N * perfData.Engines;
            double seaLevelThrust = totalIdleThrust + thrustLever * (totalMaxThrust - totalIdleThrust);
            return seaLevelThrust * (p / AtmosUtil.ISA_STD_PRES_Pa) *
                   Math.Sqrt(T / AtmosUtil.ISA_STD_TEMP_K);
        }

        public static double ConvertTasToGs(double tas, double fpa_rads, double hwind)
        {
            double tas_parallel = tas * Math.Cos(fpa_rads);
            return tas_parallel - hwind;
        }

        public static double ConvertGsToTas(double gs, double fpa_rads, double hwind)
        {
            double tas_parallel = gs + hwind;
            double cosine = Math.Cos(fpa_rads);
            return cosine == 0 ? 0 : tas_parallel / Math.Cos(fpa_rads);
        }
    }
}