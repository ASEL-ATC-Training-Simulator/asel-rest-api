using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VatsimAtcTrainingSimulator.Core.Data.Converters
{
    public static class ScenarioConverters
    {
        public static void ConvertEuroscopeToSimScenario(string inFile, string outFile, Action<string> logMsg)
        {
            // Read file
            string[] filelines = File.ReadAllLines(inFile);

            List<VatsimClientPilot> pilots = new List<VatsimClientPilot>();

            VatsimClientPilot lastPilot = null;

            foreach (string line in filelines)
            {

            }
        }
    }
}
