using AviationCalcUtilNet.GeoTools;
using AviationCalcUtilNet.GeoTools.MagneticTools;
using AviationSimulation.GeoTools.GribTools;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using VatsimAtcTrainingSimulator.Core;
using VatsimAtcTrainingSimulator.Core.Data;
using VatsimAtcTrainingSimulator.Core.Data.Loaders;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft.Control.FMS;
using VatsimAtcTrainingSimulator.Core.Simulator.Aircraft.Control.FMS.Legs;
using VatsimAtcTrainingSimulator.Core.Simulator.Commands;

namespace VatsimAtcTrainingSimulator
{
    public partial class MainForm : Form
    {
        private CommandWindow commandWindow;
        private DebugWindow debugWindow;

        public MainForm()
        {
            InitializeComponent();

            commandWindow = new CommandWindow();
            commandWindow.Show(this);
            commandWindow.FormCloseEvent += commandWindow_Closed;

            debugWindow = new DebugWindow();
            debugWindow.Show(this);
            debugWindow.FormCloseEvent += debugWindow_Closed;

            clientsDataGridView.DataSource = ClientsHandler.DisplayableList;
        }

        private void settingsBtn_Click(object sender, EventArgs e)
        {
            SettingsForm sForm = new SettingsForm();
            sForm.ShowDialog();
        }

        public void logMsg(string msg)
        {
            try
            {
                debugWindow.Invoke((MethodInvoker)delegate ()
                {
                    if (debugWindow.Visible)
                    {
                        debugWindow.LogMessage(msg);
                    }
                });
            }
            catch (InvalidOperationException) { }
        }

        private void euroscopeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Create and open file dialog
            OpenFileDialog fileDialog = new OpenFileDialog()
            {
                Title = "Open ES Scenario",
                Filter = "ES Scenario File|*.txt"
            };

            if (fileDialog.ShowDialog() == DialogResult.OK)
            {
                // Read file
                string filename = fileDialog.FileName;
                ScenarioLoader.LoadEuroscopeScenario(filename, logMsg);
            }
        }

        private void pauseAllBtn_Click(object sender, EventArgs e)
        {
            if (ClientsHandler.AllPaused)
            {
                ClientsHandler.AllPaused = false;
                pauseAllBtn.Text = "Pause";
            }
            else
            {
                ClientsHandler.AllPaused = true;
                pauseAllBtn.Text = "Unpause";
            }
        }

        private void commandWindowMenuItem_CheckedChanged(object sender, EventArgs e)
        {
            if (commandWindowMenuItem.Checked)
            {
                commandWindow.Show(this);
            }
            else
            {
                commandWindow.Hide();
            }
        }

        private void commandWindow_Closed(object sender, EventArgs e)
        {
            commandWindowMenuItem.Checked = false;
        }

        private void debugWindow_Closed(object sender, EventArgs e)
        {
            debugConsoleToolStripMenuItem.Checked = false;
        }

        private void MainForm_LocationChanged(object sender, EventArgs e)
        {
            if (commandWindow.Docked && commandWindowMenuItem.Checked)
            {
                commandWindow.Left = this.Right;
                commandWindow.Top = this.Top;
            }
        }

        private void MainForm_SizeChanged(object sender, EventArgs e)
        {
            if (commandWindow.Docked && commandWindowMenuItem.Checked)
            {
                commandWindow.Height = this.Height;
            }
        }

        private void MainForm_FormClosed(object sender, FormClosedEventArgs e)
        {
            ClientsHandler.DisconnectAllClients();
        }

        private void loadSectorFileToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Create and open file dialog
            OpenFileDialog fileDialog = new OpenFileDialog()
            {
                Title = "Open Sector File",
                Filter = "Sector File|*.sct;*.sct2"
            };

            if (fileDialog.ShowDialog() == DialogResult.OK)
            {
                // Read file
                string filename = fileDialog.FileName;
                string[] filelines = File.ReadAllLines(filename);

                string sectionName = "";

                // Loop through sector file
                foreach (string line in filelines)
                {
                    // Ignore comments
                    if (line.Trim().StartsWith(";"))
                    {
                        continue;
                    }

                    if (line.StartsWith("["))
                    {
                        // Get section name
                        sectionName = line.Replace("[", "").Replace("]", "").Trim();
                    }
                    else
                    {
                        NavaidType type = NavaidType.VOR;
                        string[] items;
                        switch (sectionName)
                        {
                            case "VOR":
                                type = NavaidType.VOR;
                                goto case "AIRPORT";
                            case "NDB":
                                type = NavaidType.NDB;
                                goto case "AIRPORT";
                            case "AIRPORT":
                                type = NavaidType.AIRPORT;

                                items = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                                if (items.Length >= 4)
                                {
                                    decimal freq = 0;
                                    try
                                    {
                                        freq = Convert.ToDecimal(items[1]);
                                    }
                                    catch (Exception) { }

                                    DataHandler.AddWaypoint(new WaypointNavaid(items[0], GribUtil.ConvertSectorFileDegMinSecToDecimalDeg(items[2]), GribUtil.ConvertSectorFileDegMinSecToDecimalDeg(items[3]), "", freq, type));
                                }
                                break;
                            case "FIXES":
                                items = line.Split(' ');

                                if (items.Length >= 3)
                                {
                                    DataHandler.AddWaypoint(new Waypoint(items[0], GribUtil.ConvertSectorFileDegMinSecToDecimalDeg(items[1]), GribUtil.ConvertSectorFileDegMinSecToDecimalDeg(items[2])));
                                }
                                break;
                        }
                    }
                }
            }
        }

        private void debugConsoleToolStripMenuItem_CheckedChanged(object sender, EventArgs e)
        {
            if (debugConsoleToolStripMenuItem.Checked)
            {
                debugWindow.Show(this);
            }
            else
            {
                debugWindow.Hide();
            }
        }

        private void dataGridUpdateTimer_Tick(object sender, EventArgs e)
        {
            clientsDataGridView.Refresh();
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            bool shouldRetry = false;
            do
            {
                try
                {
                    MagneticUtil.LoadData();
                    shouldRetry = false;
                }
                catch (Exception)
                {
                    DialogResult result = MessageBox.Show("There was an error loading the WMM.COF file. Ensure that WMM.COF is placed in the 'magnetic' folder.",
                        "Error Loading Magnetic File!",
                        MessageBoxButtons.AbortRetryIgnore,
                        MessageBoxIcon.Warning);
                    if (result == DialogResult.Retry)
                    {
                        shouldRetry = true;
                    } else if (result == DialogResult.Abort)
                    {
                        Close();
                    }
                }
            } while (shouldRetry);
        }

        private void deleteAllBtn_Click(object sender, EventArgs e)
        {
            DialogResult result = MessageBox.Show("Are you sure you want to delete all aircraft?", "Delete All Aircraft?", MessageBoxButtons.YesNoCancel, MessageBoxIcon.Question);

            if (result == DialogResult.Yes)
            {
                ClientsHandler.DisconnectAllClients();
            }
        }
    }
}
