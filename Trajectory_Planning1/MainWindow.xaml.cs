    using OxyPlot;
    using OxyPlot.Axes;
    using OxyPlot.Series;
    using OxyPlot.Wpf;
    using System;
    using System.Collections.Generic;          // <- needed for List<>
    using System.IO.Ports;
    using System.Windows;
using System.Windows.Controls;
    using System.Windows.Threading;
    using System.Windows.Threading;
    using static System.Runtime.InteropServices.JavaScript.JSType;

    namespace Trajectory_Planning1
    {
        public partial class MainWindow : Window
        {
            private SerialPort serialPort;
            private bool isConnected = false;
            private PlotModel model;
            private LineSeries armSeries;
            private LineSeries trailSeries;
            private List<LineSeries> ghostArms = new List<LineSeries>();
            private LineSeries segment1, segment2, segment3;
            private DispatcherTimer timer;
            private double[] qxArray, qyArray, orientasiArray;
            private int space = 0; // 0 = joint space, 1 = workspace
            // Motion data
            private double[] teta1Array, teta2Array, teta3Array;
            private int n = 0;
            private int totalPoints = 0;
            private double totalTime = 0;
            private int jenis = 0;  // 0 = Time/Path, 1 = Time/Track
            // === Trajectory UI ===
            private List<(TextBox t1, TextBox t2, TextBox t3, Button runBtn)> trajectoryRowsList 
            = new List<(TextBox, TextBox, TextBox, Button)>();
        public MainWindow()
            {

                InitializeComponent();
                InitializePlot();
                cmbPorts.ItemsSource = SerialPort.GetPortNames();
                if (cmbPorts.Items.Count > 0)
                    cmbPorts.SelectedIndex = 0;
                timer = new DispatcherTimer();
                timer.Tick += Timer_Tick;
            }
            private void InitializeSerialPort()
            {
                serialPort = new SerialPort();
            }
            private void ComboBox_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
            {

                // Optional: show selected port
                if (cmbPorts.SelectedItem != null)
                {
                    string selectedPort = cmbPorts.SelectedItem.ToString();
                    Console.WriteLine("Selected port: " + selectedPort);
                }
            }

            private void btnConnect_Click(object sender, RoutedEventArgs e)
            {
                try
                {
                    if (!isConnected)
                    {
                        if (cmbPorts.SelectedItem == null)
                        {
                            MessageBox.Show("Please select a COM port first.");
                            return;
                        }

                        string portName = cmbPorts.SelectedItem.ToString();
                        serialPort = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
                        serialPort.Open();

                        isConnected = true;
                        btnConnect.Content = "Close Port";
                        MessageBox.Show($"Connected to {portName} successfully!", "Serial", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                    else
                    {
                        if (serialPort != null && serialPort.IsOpen)
                            serialPort.Close();

                        isConnected = false;
                        btnConnect.Content = "Open Port";
                        MessageBox.Show("Port closed.", "Serial", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error: " + ex.Message, "Serial Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
            private async void btnConnectOrig_Click(object sender, RoutedEventArgs e)
            {
                if (serialPort == null || !serialPort.IsOpen)
                {
                    MessageBox.Show("Serial port not open!");
                    return;
                }

                // The predefined servo commands
                string[] commands =
                {
            "#0 P1567 T2000",
            "#1 P2250 T2000",
            "#2 P1465 T2000",
            "#3 P1520 T2000",
            "#4 P1500 T2000"
        };

                try
                {
                    foreach (string cmd in commands)
                    {
                        serialPort.WriteLine(cmd + "\r\n");
                        Console.WriteLine($"Sent: {cmd}");
                        await Task.Delay(2000); // small delay between commands
                    }

                    MessageBox.Show("Servos moved to origin positions.", "Origin", MessageBoxButton.OK, MessageBoxImage.Information);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error sending servo commands: " + ex.Message);
                }
            }

            private void InitializePlot()
            {
                model = new PlotModel { Title = "Robot Motion Area" };

                // X axis
                model.Axes.Add(new LinearAxis
                {
                    Position = AxisPosition.Bottom,
                    Minimum = -30,
                    Maximum = 30,
                    MajorStep = 5,
                    MinorStep = 1,
                    AxislineStyle = LineStyle.Solid,
                    AxislineColor = OxyColors.Black,
                    Title = "X (cm)",
                    MajorGridlineStyle = LineStyle.Solid,
                    MinorGridlineStyle = LineStyle.Dot,
                    MajorGridlineColor = OxyColors.LightGray,
                    MinorGridlineColor = OxyColors.LightGray
                });

                // Y axis (example: 0..60)
                model.Axes.Add(new LinearAxis
                {
                    Position = AxisPosition.Left,
                    Minimum = 0,
                    Maximum = 60,
                    MajorStep = 10,
                    MinorStep = 2,
                    AxislineStyle = LineStyle.Solid,
                    AxislineColor = OxyColors.Black,
                    Title = "Y (cm)",
                    MajorGridlineStyle = LineStyle.Solid,
                    MinorGridlineStyle = LineStyle.Dot,
                    MajorGridlineColor = OxyColors.LightGray,
                    MinorGridlineColor = OxyColors.LightGray
                });

                // Current arm (dark)
                armSeries = new LineSeries
                {
                    Color = OxyColors.DarkGray,
                    StrokeThickness = 3,
                    Title = "Current Arm"
                };
                model.Series.Add(armSeries);

                // Trail (red points)
                trailSeries = new LineSeries
                {
                    Color = OxyColors.Red,
                    StrokeThickness = 2,
                    Title = "End Effector Trail",
                    LineStyle = LineStyle.Solid,
                    MarkerType = MarkerType.Circle,
                    MarkerSize = 2,
                    MarkerFill = OxyColors.Red
                };
                model.Series.Add(trailSeries);

                // Separate colored segments for current arm
                segment1 = new LineSeries { Color = OxyColors.Blue, StrokeThickness = 3 };
                segment2 = new LineSeries { Color = OxyColors.Green, StrokeThickness = 3 };
                segment3 = new LineSeries { Color = OxyColors.Orange, StrokeThickness = 3 };

                model.Series.Add(segment1);
                model.Series.Add(segment2);
                model.Series.Add(segment3);

                plotView.Model = model;
            }

            // ArmDraw expects angles in DEGREES (converts to radians internally)
            private void ArmDraw(double degT1, double degT2, double degT3)
            {
                // read link lengths from UI (safe parse)
                if (!double.TryParse(txtA1.Text, out double a1)) a1 = 100;
                if (!double.TryParse(txtA2.Text, out double a2)) a2 = 80;
                if (!double.TryParse(txtA3.Text, out double a3)) a3 = 60;

                // convert degrees -> radians
                double t1 = degT1 * Math.PI / 180.0;
                double t2 = degT2 * Math.PI / 180.0;
                double t3 = degT3 * Math.PI / 180.0;

                // forward kinematics
                double kx = a1 * Math.Cos(t1);
                double ky = a1 * Math.Sin(t1);

                double px = kx + a2 * Math.Cos(t1 + t2);
                double py = ky + a2 * Math.Sin(t1 + t2);

                double qx = px + a3 * Math.Cos(t1 + t2 + t3);
                double qy = py + a3 * Math.Sin(t1 + t2 + t3);

                // --- create ghost arm (semi-transparent) ---
                var ghost = new LineSeries
                {
                    Color = OxyColor.FromAColor(60, OxyColors.SkyBlue), // translucent
                    StrokeThickness = 1.2,
                    LineStyle = LineStyle.Solid
                };
                ghost.Points.Add(new DataPoint(0, 0));
                ghost.Points.Add(new DataPoint(kx, ky));
                ghost.Points.Add(new DataPoint(px, py));
                ghost.Points.Add(new DataPoint(qx, qy));

                model.Series.Add(ghost);
                ghostArms.Add(ghost);

                // --- update current arm with 3 colored segments ---
                segment1.Points.Clear();
                segment1.Points.Add(new DataPoint(0, 0));
                segment1.Points.Add(new DataPoint(kx, ky));

                segment2.Points.Clear();
                segment2.Points.Add(new DataPoint(kx, ky));
                segment2.Points.Add(new DataPoint(px, py));

                segment3.Points.Clear();
                segment3.Points.Add(new DataPoint(px, py));
                segment3.Points.Add(new DataPoint(qx, qy));

                // --- append to trail (end-effector path) ---
                trailSeries.Points.Add(new DataPoint(qx, qy));

                // limit number of ghost arms to avoid memory blow-up
                const int maxGhost = 120;
                if (ghostArms.Count > maxGhost)
                {
                    var oldest = ghostArms[0];
                    model.Series.Remove(oldest);
                    ghostArms.RemoveAt(0);
                }

                model.InvalidatePlot(true);
            }

            private void btnClearPath_Click(object sender, RoutedEventArgs e)
            {
                // Clear the end-effector trail
                trailSeries.Points.Clear();

                // Remove and clear ghost arms
                foreach (var g in ghostArms)
                    model.Series.Remove(g);
                ghostArms.Clear();

                // Clear current arm segments
                segment1.Points.Clear();
                segment2.Points.Clear();
                segment3.Points.Clear();

                model.InvalidatePlot(true);
            }


            private void InverseKinematic1(double iqx, double iqy, double iorientasi)
            {
                // Convert orientation to radians
                double phi = iorientasi * Math.PI / 180.0;

                if (!double.TryParse(txtA1.Text, out double a1)) a1 = 100;
                if (!double.TryParse(txtA2.Text, out double a2)) a2 = 80;
                if (!double.TryParse(txtA3.Text, out double a3)) a3 = 60;

                // Wrist position
                double wx = iqx - a3 * Math.Cos(phi);
                double wy = iqy - a3 * Math.Sin(phi);

                // Compute D = cos(θ2)
                double D = (wx * wx + wy * wy - a1 * a1 - a2 * a2) / (2 * a1 * a2);
                D = Math.Clamp(D, -1.0, 1.0);

                // === Choose elbow-UP configuration ===
                double t2 = -Math.Acos(D);   // NEGATIVE to keep arm in upper half-plane

                // Compute θ1 for elbow-up
                double t1 = Math.Atan2(wy, wx) - Math.Atan2(a2 * Math.Sin(t2), a1 + a2 * Math.Cos(t2));

                // Compute θ3 for desired orientation
                double t3 = phi - t1 - t2;

                // Convert to degrees
                t1 *= 180.0 / Math.PI;
                t2 *= 180.0 / Math.PI;
                t3 *= 180.0 / Math.PI;


                ArmDraw(t1, t2, t3);
            }

            // Calculate button: fill teta arrays and listbox
            private void btnCalculate_Click(object sender, RoutedEventArgs e)
            {
                try
                {
                    space = 0;

                    totalPoints = Math.Max(1, int.Parse(txtTitik.Text)); // ensure >=1

                    double t1_0 = double.Parse(txtInit1.Text);
                    double t2_0 = double.Parse(txtInit2.Text);
                    double t3_0 = double.Parse(txtInit3.Text);

                    double t1_1 = double.Parse(txtFinal1.Text);
                    double t2_1 = double.Parse(txtFinal2.Text);
                    double t3_1 = double.Parse(txtFinal3.Text);

                    lbJointList.Items.Clear();
                    teta1Array = new double[totalPoints + 1];
                    teta2Array = new double[totalPoints + 1];
                    teta3Array = new double[totalPoints + 1];

                    for (int i = 0; i <= totalPoints; i++)
                    {
                        double alpha = (double)i / totalPoints;
                        teta1Array[i] = t1_0 + (t1_1 - t1_0) * alpha;
                        teta2Array[i] = t2_0 + (t2_1 - t2_0) * alpha;
                        teta3Array[i] = t3_0 + (t3_1 - t3_0) * alpha;

                        lbJointList.Items.Add($"{i,3} | {teta1Array[i],6:0.0} | {teta2Array[i],6:0.0} | {teta3Array[i],6:0.0}");
                    }

                    MessageBox.Show("Joint interpolation complete!", "Success", MessageBoxButton.OK, MessageBoxImage.Information);
                    PopulateEditableTrajectoryRows();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error in Calculate: " + ex.Message, "Input Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
            private void btnWorkCalculate_Click(object sender, RoutedEventArgs e)
            {
                try
                {
                    int n = int.Parse(txtTitik.Text);

                    double qx0 = double.Parse(txtWorkInit1.Text);
                    double qy0 = double.Parse(txtWorkInit2.Text);
                    double ori0 = double.Parse(txtWorkInit3.Text);

                    double qx1 = double.Parse(txtWorkFinal1.Text);
                    double qy1 = double.Parse(txtWorkFinal2.Text);
                    double ori1 = double.Parse(txtWorkFinal3.Text);

                    lbWorkList.Items.Clear();

                    // use global arrays so Timer_Tick and InverseKinematic1 can access them
                    qxArray = new double[n + 1];
                    qyArray = new double[n + 1];
                    orientasiArray = new double[n + 1];

                    for (int i = 0; i <= n; i++)
                    {
                        double alpha = (double)i / n;
                        qxArray[i] = qx0 + (qx1 - qx0) * alpha;
                        qyArray[i] = qy0 + (qy1 - qy0) * alpha;
                        orientasiArray[i] = ori0 + (ori1 - ori0) * alpha;

                        lbWorkList.Items.Add($"{i,3} | {qxArray[i],6:0.0} | {qyArray[i],6:0.0} | {orientasiArray[i],6:0.0}");
                    }


                    MessageBox.Show("Workspace interpolation complete!", "Success", MessageBoxButton.OK, MessageBoxImage.Information);
                PopulateWorkspaceTrajectoryRows();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error: " + ex.Message, "Input Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }

            // Run button: starts the timer (mimics OnBnClickedButton3)
            private void btnRun_Click(object sender, RoutedEventArgs e)
            {
                try
                {
                    space = 0;

                    if (teta1Array == null || teta1Array.Length == 0)
                    {
                        MessageBox.Show("Please press Calculate first to generate trajectory.", "Run", MessageBoxButton.OK, MessageBoxImage.Information);
                        return;
                    }

                    // totalTime and totalPoints
                    totalTime = double.Parse(txtWaktu.Text);    // treat as milliseconds (like SetTimer)
                    totalPoints = teta1Array.Length - 1;

                    n = 0;

                    // jenis from radio buttons (rbTimePath / rbTimeTrack)
                    jenis = (rbTimePath.IsChecked == true) ? 0 : 1;

                    double interval = (jenis == 0) ? totalTime : (totalTime / (double)totalPoints);

                    // sanity: interval must be > 0
                    if (interval <= 0) interval = 100; // default 100 ms

                    timer.Interval = TimeSpan.FromMilliseconds(interval);
                    timer.Start();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error in Run: " + ex.Message, "Run Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
            private void btnWorkRun_Click(object sender, RoutedEventArgs e)
            {
                try
                {
                    if (qxArray == null || qxArray.Length == 0)
                    {
                        MessageBox.Show("Please press Work Calculate first.", "Run", MessageBoxButton.OK, MessageBoxImage.Information);
                        return;
                    }

                    n = 0;
                    space = 1; // workspace mode

                    int totalPoints = qxArray.Length - 1;
                    double totalTime = double.Parse(txtWaktu.Text);

                    // interval in milliseconds
                    double interval = (rbTimePath.IsChecked == true) ? totalTime : (totalTime / totalPoints);
                    if (interval <= 0) interval = 100;

                    timer.Interval = TimeSpan.FromMilliseconds(interval);
                    timer.Start();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error in Work Run: " + ex.Message, "Run Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }


            // Timer tick: animate
            private void Timer_Tick(object sender, EventArgs e)
            {
                try
                {
                    if (space == 0 && teta1Array != null)
                    {
                        ArmDraw(teta1Array[n], teta2Array[n], teta3Array[n]);
                    }
                    else if (space == 1 && qxArray != null)
                    {
                        InverseKinematic1(qxArray[n], qyArray[n], orientasiArray[n]);
                    }

                    n++;

                    if ((space == 0 && n > totalPoints) || (space == 1 && n > qxArray.Length - 1))
                        timer.Stop();
                }
                catch
                {
                    timer.Stop();
                }
            }
        private void SingleRun_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is int index)
            {
                var row = trajectoryRowsList[index];

                if (int.TryParse(row.t1.Text, out int pwm1) &&
                    int.TryParse(row.t2.Text, out int pwm2) &&
                    int.TryParse(row.t3.Text, out int pwm3))
                {
                    // Send to servos immediately
                    if (serialPort != null && serialPort.IsOpen)
                    {
                        string cmd = $"#0 P{pwm1} T2000 #2 P{pwm2} T2000 #3 P{pwm3} T2000\r\n";
                        serialPort.WriteLine(cmd + "\r\n");
                        Console.WriteLine($"Sent: {cmd}");
                    }
                }
                else
                {
                    MessageBox.Show("Invalid numeric values in row " + index);
                }
            }
        }

        private readonly Dictionary<int, (int min, int max)> servoRanges = new Dictionary<int, (int min, int max)>
        {
            { 0, (635, 2500) }, // slider0
            { 2, (500, 2430) }, // slider2
            { 3, (650, 2390) }, // slider3
        };
        private int AngleToInterp(int servoIndex, double angle)
        {
            if (!servoRanges.TryGetValue(servoIndex, out var r))
                return 1500;

            int min = r.min; // PWM for 0 deg
            int max = r.max; // PWM for 180 deg
            double pwm = 0;

            if (servoIndex == 0)
            {
                // Servo 0: 0..180
                angle = Math.Clamp(angle, 0, 180);
                pwm = max - (angle / 180.0) * (max - min); // flip
            }
            else // servo 2 or 3
            {
                // Logical -90..+90 → physical 0..180
                angle = Math.Clamp(angle, -90, 90);
                double physicalAngle = angle + 90; // now 0..180
                pwm = max - (physicalAngle / 180.0) * (max - min); // flip
            }

            return (int)Math.Round(pwm);
        }
        private void PopulateEditableTrajectoryRows()
        {
            trajectoryRows.Children.Clear();
            trajectoryRowsList.Clear();

            if (teta1Array == null || teta1Array.Length == 0)
                return;

            for (int i = 0; i < teta1Array.Length; i++)
            {
                var panel = new StackPanel
                {
                    Orientation = Orientation.Horizontal,
                    Margin = new Thickness(0, 2, 0, 2)
                };

                // Convert angles to interpolation (PWM) values
                int pwm1 = AngleToInterp(0, teta1Array[i]);
                int pwm2 = AngleToInterp(2, teta2Array[i]);
                int pwm3 = AngleToInterp(3, teta3Array[i]);

                // textboxes show interpolation values
                var txt1 = new TextBox { Width = 60, Text = pwm1.ToString(), Margin = new Thickness(2) };
                var txt2 = new TextBox { Width = 60, Text = pwm2.ToString(), Margin = new Thickness(2) };
                var txt3 = new TextBox { Width = 60, Text = pwm3.ToString(), Margin = new Thickness(2) };

                var btnRun = new Button
                {
                    Content = "Run",
                    Width = 50,
                    Height = 24,
                    Margin = new Thickness(4, 0, 0, 0),
                    Tag = i
                };
                btnRun.Click += SingleRun_Click;

                panel.Children.Add(txt1);
                panel.Children.Add(txt2);
                panel.Children.Add(txt3);
                panel.Children.Add(btnRun);

                trajectoryRows.Children.Add(panel);
                trajectoryRowsList.Add((txt1, txt2, txt3, btnRun));
            }
        }
        private void PopulateWorkspaceTrajectoryRows()
        {
            trajectoryRows.Children.Clear();
            trajectoryRowsList.Clear();

            if (qxArray == null || qxArray.Length == 0)
                return;

            for (int i = 0; i < qxArray.Length; i++)
            {
                double iqx = qxArray[i];
                double iqy = qyArray[i];
                double iori = orientasiArray[i];

                // Compute joint angles (without drawing)
                var angles = WorkspaceToJointAngles(iqx, iqy, iori);

                // Convert joint angles to PWM
                int pwm1 = AngleToInterp(0, angles.t1);
                int pwm2 = AngleToInterp(2, angles.t2);
                int pwm3 = AngleToInterp(3, angles.t3);

                // Create UI row
                var panel = new StackPanel
                {
                    Orientation = Orientation.Horizontal,
                    Margin = new Thickness(0, 2, 0, 2)
                };

                var txt1 = new TextBox { Width = 60, Text = pwm1.ToString(), Margin = new Thickness(2) };
                var txt2 = new TextBox { Width = 60, Text = pwm2.ToString(), Margin = new Thickness(2) };
                var txt3 = new TextBox { Width = 60, Text = pwm3.ToString(), Margin = new Thickness(2) };

                var btnRun = new Button
                {
                    Content = "Run",
                    Width = 50,
                    Height = 24,
                    Margin = new Thickness(4, 0, 0, 0),
                    Tag = i
                };
                btnRun.Click += SingleRun_Click;

                panel.Children.Add(txt1);
                panel.Children.Add(txt2);
                panel.Children.Add(txt3);
                panel.Children.Add(btnRun);

                trajectoryRows.Children.Add(panel);
                trajectoryRowsList.Add((txt1, txt2, txt3, btnRun));
            }
        }

        // Helper: computes joint angles for given workspace coordinates
        private (double t1, double t2, double t3) WorkspaceToJointAngles(double iqx, double iqy, double iorientasi)
        {
            double phi = iorientasi * Math.PI / 180.0;
            if (!double.TryParse(txtA1.Text, out double a1)) a1 = 100;
            if (!double.TryParse(txtA2.Text, out double a2)) a2 = 80;
            if (!double.TryParse(txtA3.Text, out double a3)) a3 = 60;

            double wx = iqx - a3 * Math.Cos(phi);
            double wy = iqy - a3 * Math.Sin(phi);

            double D = (wx * wx + wy * wy - a1 * a1 - a2 * a2) / (2 * a1 * a2);
            D = Math.Clamp(D, -1.0, 1.0);

            double t2 = -Math.Acos(D);
            double t1 = Math.Atan2(wy, wx) - Math.Atan2(a2 * Math.Sin(t2), a1 + a2 * Math.Cos(t2));
            double t3 = phi - t1 - t2;

            // Convert to degrees, no clamp here
            double degT1 = t1 * 180.0 / Math.PI;
            double degT2 = t2 * 180.0 / Math.PI;
            double degT3 = t3 * 180.0 / Math.PI;

            return (degT1, degT2, degT3);
        }
        private async void btnRunAll_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if ((space == 0 && teta1Array == null) || (space == 1 && qxArray == null))
                {
                    MessageBox.Show("No trajectory data available!", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                n = 0; // reset index
                int steps = (space == 0) ? totalPoints : qxArray.Length - 1;
                if (steps <= 0) steps = 1;

                double totalTime = double.Parse(txtWaktu.Text);
                double interval = (rbTimePath.IsChecked == true) ? totalTime : (totalTime / steps);
                if (interval < 1) interval = 1;

                int T = (int)Math.Round(interval); // same T for servo as in timer

                for (n = 0; n <= steps; n++)
                {
                    int pwm0, pwm2, pwm3;

                    if (space == 0 && teta1Array != null)
                    {
                        pwm0 = AngleToInterp(0, teta1Array[n]);
                        pwm2 = AngleToInterp(2, teta2Array[n]);
                        pwm3 = AngleToInterp(3, teta3Array[n]);
                    }
                    else if (space == 1 && qxArray != null)
                    {
                        var angles = WorkspaceToJointAngles(qxArray[n], qyArray[n], orientasiArray[n]);
                        pwm0 = AngleToInterp(0, angles.t1);
                        pwm2 = AngleToInterp(2, angles.t2);
                        pwm3 = AngleToInterp(3, angles.t3);
                    }
                    else
                    {
                        continue; // safety
                    }

                    // Send command to servo
                    if (serialPort != null && serialPort.IsOpen)
                    {
                        string cmd = $"#0 P{pwm0} T{T} #2 P{pwm2} T{T} #3 P{pwm3} T{T}\r\n";
                        serialPort.WriteLine(cmd);
                    }

                    // Draw/update if needed (optional)
                    if (space == 0)
                        ArmDraw(teta1Array[n], teta2Array[n], teta3Array[n]);
                    else
                        InverseKinematic1(qxArray[n], qyArray[n], orientasiArray[n]);

                    await Task.Delay(T); // wait before next step
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error in Run All: " + ex.Message, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }

    }
}
