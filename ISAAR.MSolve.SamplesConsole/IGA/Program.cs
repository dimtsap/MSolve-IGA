using ISAAR.MSolve.Analyzers;
using ISAAR.MSolve.IGA;
using ISAAR.MSolve.IGA.Entities;
using ISAAR.MSolve.IGA.Entities.Loads;
using ISAAR.MSolve.IGA.Problems.Structural;
using ISAAR.MSolve.IGA.Problems.SupportiveClasses;
using ISAAR.MSolve.IGA.Readers;
using ISAAR.MSolve.Numerical.LinearAlgebra;
using ISAAR.MSolve.Solvers.Skyline;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace ISAAR.MSolve.SamplesConsole.IGA
{
    class Program
    {
        private static void SolveIsogeometricModelExample()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            model.PatchesDictionary.Add(1, new Patch() { ID = 1 });
            //ModelReader.CreateModelFromFile(model, "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\Cantilever2D biquadratic.txt");
            model.ConnectDataStructures();

            // Forces and Boundary Conditions
            //model.ControlPointsDictionary[0].Constrains.Add(DOFType.X);
            //model.Loads.Add(new ControlPointLoad() {Amount =-100 , ControlPoint=model.ControlPoints[7], DOF=DOFType.Y});
            //model.PatchesDictionary[0].FacesDictionary[0].BoundaryConditions.Add()
            //model.CalculateForces();

            var linearSystem = new SkylineLinearSystem(1, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            //Log den exoun ginei

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
        }

        private static void SolveIsogeometricPlateWithHoleIGAFEM()
        {
            // Model
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\PlateWithHoleIGAFEM.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();

            // Forces and Boundary Conditions
            // Loading Conditions - Pressure
            Value plateNeumannValue = delegate (double x, double y, double z) 
            {
                double r = Math.Sqrt(x * x + y * y);
                double theta = Math.Atan(y / x);

                double c2t = Math.Cos(2 * theta);
                double c4t = Math.Cos(4 * theta);
                double s2t = Math.Sin(2 * theta);
                double s4t = Math.Sin(4 * theta);
                double fac1 = Math.Pow((1 / r),2);
                double fac2 = fac1 * fac1;
                double[] exactStress = new double[3];
                exactStress[0] = 1 - fac1 * (1.5 * c2t + c4t) + 1.5 * fac2 * c4t;
                exactStress[1] = -fac1 * (0.5 * c2t - c4t) - 1.5 * fac2 * c4t;
                exactStress[2] = -fac1 * (0.5 * s2t + s4t) + 1.5 * fac2 * s4t;
                return exactStress;
            };
            
            model.PatchesDictionary[0].EdgesDictionary[3].LoadingConditions.Add(new NeumannBoundaryCondition(plateNeumannValue));

            // Boundary Conditions - Dirichlet
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[0].ControlPointsDictionary.Values)
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);

            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[1].ControlPointsDictionary.Values)
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);

            model.ConnectDataStructures();

            // Solvers
            var linearSystem = new SkylineLinearSystem(0, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
            
        }

        private static void SolveIsogeometricLShaped()
        {
            // Model
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\LShaped.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();

            // Forces and Boundary Conditions
            // Loading Conditions - Pressure
            Value plateNeumannValue = delegate (double x, double y, double z)
            {
                if (x == 0)
                    return new double[] { 1, 0, 0 };
                return new double[] { 0, 0, 0 };
            };

            model.PatchesDictionary[0].EdgesDictionary[0].LoadingConditions.Add(new NeumannBoundaryCondition(plateNeumannValue));
            

            // Boundary Conditions - Dirichlet
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[2].ControlPointsDictionary.Values)
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);

            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[3].ControlPointsDictionary.Values)
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);

            model.ConnectDataStructures();
            //model.AssignLoads();
            
            // Solvers
            var linearSystem = new SkylineLinearSystem(0, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
        }

        private static void SolveIsogeometricBeam3D()
        {
            // Model
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\Beam3D.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();

            // Forces and Boundary Conditions
            // Loading Conditions - Pressure
            //Value plateNeumannValue = delegate (double x, double y, double z) { return new double[] { 0, -100, 0 }; };
            //model.PatchesDictionary[0].FacesDictionary[1].LoadingConditions.Add(new NeumannBoundaryCondition(plateNeumannValue));
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].FacesDictionary[1].ControlPointsDictionary.Values)
                model.Loads.Add(new Load() { Amount = -100, ControlPoint = model.ControlPoints[controlPoint.ID], DOF = DOFType.Z });

            // Boundary Conditions - Dirichlet
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].FacesDictionary[0].ControlPointsDictionary.Values)
            {
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Z);
            }



            model.ConnectDataStructures();

            // Solvers
            var linearSystem = new SkylineLinearSystem(0, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
        }

        private static void SolveIsogeometricCantilever2D()
        {
            // Model
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\Cantilever2D.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();

            // Forces and Boundary Conditions
            // Loading Conditions - Pressure
            Value plateNeumannValue = delegate (double x, double y, double z)
            {
                return new double[] { 0, -100, 0 };
            };

            model.PatchesDictionary[0].EdgesDictionary[1].LoadingConditions.Add(new NeumannBoundaryCondition(plateNeumannValue));


            // Boundary Conditions - Dirichlet
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[0].ControlPointsDictionary.Values)
            {
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);
            }
                

            model.ConnectDataStructures();

            // Solvers
            var linearSystem = new SkylineLinearSystem(0, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
        }


        #region To be reviewed
        private static void SolveIsogeometricPlateWithHole()
        {
            // Model
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\PlateWithHole.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();

            // Forces and Boundary Conditions
            // Loading Conditions - Pressure
            //Value pressureValue = delegate (GaussLegendrePoint3D gaussPoint) { return 0.5; };
            //model.PatchesDictionary[0].EdgesDictionary[0].LoadingConditions.Add(new PressureBoundaryCondition(pressureValue));
            //model.PatchesDictionary[0].EdgesDictionary[1].LoadingConditions.Add(new PressureBoundaryCondition(pressureValue));
            //model.PatchesDictionary[0].EdgesDictionary[3].LoadingConditions.Add(new PressureBoundaryCondition(pressureValue));

            // Boundary Conditions - Dirichlet
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[2].ControlPointsDictionary.Values)
            {
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);
            }
            model.AssignLoads();

            // Solvers
            var linearSystem = new SkylineLinearSystem(0, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
        }

        private static void SolveIsogeometricPlainStarinSquareMixedBC()
        {
            // Model
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\PlainStrainSquare.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();

            // Forces and Boundary Conditions
            // Loading Conditions - Pressure
            //Value pressureValue = delegate (GaussLegendrePoint3D gaussPoint) { return 0.3; };
            //model.PatchesDictionary[0].EdgesDictionary[1].LoadingConditions.Add(new PressureBoundaryCondition(pressureValue));
            //model.PatchesDictionary[0].EdgesDictionary[3].LoadingConditions.Add(new PressureBoundaryCondition(pressureValue));

            // Boundary Conditions - Dirichlet
            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[0].ControlPointsDictionary.Values)
            {
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);
            }

            foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[2].ControlPointsDictionary.Values)
            {
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.X);
                model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(DOFType.Y);
            }


            // Solvers
            var linearSystem = new SkylineLinearSystem(1, model.PatchesDictionary[0].Forces);
            SolverFBSubstitution solver = new SolverFBSubstitution(linearSystem);
            ProblemStructural provider = new ProblemStructural(model);
            LinearAnalyzer analyzer = new LinearAnalyzer(solver, linearSystem);
            StaticAnalyzer parentAnalyzer = new StaticAnalyzer(provider, analyzer, linearSystem);

            parentAnalyzer.BuildMatrices();
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
        }

        private static void ReadIsogeometricModelOnePatch()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-One Patch-Draft.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelOnePatch3D()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-3D-OnePatch.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelMultiplePatchesContinuous3D()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            modelCreator.SplitToContinuousPatches = true;
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-3D-Multiple Patches -Continuous.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelOnePatchContinuous()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            modelCreator.SplitToContinuousPatches = true;
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-One Patch-Continuous Patches - Draft.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelOnePatchThreeContinuous()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            modelCreator.SplitToContinuousPatches = true;
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-One Patch - Three Continuous.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelOnePatchThreeContinuousDegree4()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            modelCreator.SplitToContinuousPatches = true;
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-One Patch - Three Continuous - Degree4.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelOnePatchFourContinuousDegree4()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            modelCreator.SplitToContinuousPatches = true;
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-One Patch - Four Continuous - Degree4.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelMultiplePatches()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-Multiple Patches -Draft.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }

        private static void ReadIsogeometricModelMultiplePatchesContinuous()
        {
            VectorExtensions.AssignTotalAffinityCount();
            Model model = new Model();
            ModelCreator modelCreator = new ModelCreator(model);
            modelCreator.SplitToContinuousPatches = true;
            string filename = "C:\\Users\\user\\Desktop\\MSolve-master\\ISAAR.MSolve.SamplesConsole\\IGA\\Input Files\\InputFile-Multiple Patches -Continuous -Draft.txt";
            IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filename);
            modelReader.CreateModelFromFile();
        }
        #endregion

        static void Main(string[] args)
        {
            //SolveIsogeometricLShaped();
            //SolveIsogeometricPlateWithHoleIGAFEM();
            SolveIsogeometricCantilever2D();
        }
    }
}
