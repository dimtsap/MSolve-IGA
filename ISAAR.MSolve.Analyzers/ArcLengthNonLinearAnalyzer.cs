using ISAAR.MSolve.Analyzers.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ISAAR.MSolve.Logging.Interfaces;
using ISAAR.MSolve.Solvers.Interfaces;
using ISAAR.MSolve.Numerical.LinearAlgebra.Interfaces;
using ISAAR.MSolve.Numerical.LinearAlgebra;
using ISAAR.MSolve.Solvers.Skyline;

namespace ISAAR.MSolve.Analyzers
{
    class ArcLengthNonLinearAnalyzer : IAnalyzer
    {
        #region IAnalyzer Fields
        private readonly ILinearSystem[] linearSystems;
        private readonly INonLinearSubdomainUpdater[] subdomainUpdaters;
        private readonly ISubdomainGlobalMapping[] mappings;
        private readonly Dictionary<int, ILinearAnalyzerLogFactory> logFactories = new Dictionary<int, ILinearAnalyzerLogFactory>();
        private readonly Dictionary<int, IAnalyzerLog[]> logs = new Dictionary<int, IAnalyzerLog[]>();
        private INonLinearParentAnalyzer parentAnalyzer = null;
        private readonly ISolver solver;
        private readonly INonLinearProvider provider;
        #endregion

        #region Newton-Raphson Fields possibly not needed
        private readonly int increments;
        private readonly int totalDOFs;
        private readonly int maxSteps = 120;
        private readonly int stepsForMatrixRebuild =500;
        #endregion


        private int cycle=0;
        private Boolean active=true;

        private readonly double tolerance = 1e-5;
        private readonly int optimumIterations = 5;
        private readonly int maximumIterations = 10;
        private readonly Boolean fixedStep = false;
        private readonly double maximumFactor = 1.0e20;
        private readonly double maximumLambda = 1.0e20;
        private readonly double  totalFactor=1;
        private readonly double factor=1;
        private double previousDLambda=1;
        private double lambda=1;

        private double rhsNorm;
        
        private readonly Dictionary<int, IVector> rhs = new Dictionary<int, IVector>();
        private readonly Dictionary<int, Vector> u = new Dictionary<int, Vector>();
        private readonly Dictionary<int, Vector> du = new Dictionary<int, Vector>();
        private readonly Dictionary<int, Vector> uPlusdu = new Dictionary<int, Vector>();
        private readonly Dictionary<int, Vector> previousDu = new Dictionary<int, Vector>();
        private readonly Vector globalRHS;
        

        public ArcLengthNonLinearAnalyzer(ISolver solver, ILinearSystem[] linearSystems, INonLinearSubdomainUpdater[] subdomainUpdaters, ISubdomainGlobalMapping[] mappings,
            INonLinearProvider provider, int increments, int totalDOFs)
        {
            this.solver = solver;
            this.subdomainUpdaters = subdomainUpdaters;
            this.mappings = mappings;
            this.linearSystems = linearSystems;
            this.provider = provider;
            this.increments = increments;
            this.totalDOFs = totalDOFs;
            this.globalRHS = new Vector(totalDOFs);


            InitializeInternalVectors();
        }

        public ArcLengthNonLinearAnalyzer(ISolver solver, ILinearSystem linearSystem, INonLinearSubdomainUpdater[] subdomainUpdaters, ISubdomainGlobalMapping[] mappings,
            INonLinearProvider provider, int increments, int totalDOFs)
        {
            this.solver = solver;
            this.subdomainUpdaters = subdomainUpdaters;
            this.mappings = mappings;
            this.linearSystems = new[] { linearSystem };
            this.provider = provider;
            this.increments = increments;
            this.totalDOFs = totalDOFs;
            this.globalRHS = new Vector(totalDOFs);

            InitializeInternalVectors();
        }

        private void InitializeLogs()
        {
            logs.Clear();
            foreach (int id in logFactories.Keys) logs.Add(id, logFactories[id].CreateLogs());
        }

        private void StoreLogResults(DateTime start, DateTime end)
        {
            foreach (int id in logs.Keys)
                foreach (var l in logs[id])
                    l.StoreResults(start, end, linearSystems[id].Solution);
        }

        public Dictionary<int, ILinearAnalyzerLogFactory> LogFactories { get { return logFactories; } }

        
        #region IAnalyzer Members

        public IAnalyzer ChildAnalyzer
        {
            get { return null; }
            set { throw new InvalidOperationException("Arc-Length analyzer cannot contain an embedded analyzer"); }
        }

        public Dictionary<int, IAnalyzerLog[]> Logs { get { return logs; } }

        public IAnalyzer ParentAnalyzer
        {
            get { return (IAnalyzer)parentAnalyzer; }
            set { parentAnalyzer = (INonLinearParentAnalyzer)value; }
        }

        public void BuildMatrices()
        {
            if (parentAnalyzer == null)
                throw new InvalidOperationException("This Arc-Length non-linear analyzer has no parent.");

            parentAnalyzer.BuildMatrices();
        }

        public void Initialize()
        {
            solver.Initialize();
        }

        public void Solve()
        {        
            //while (active)
            //{
            //    cycle++;

            //    foreach (ILinearSystem subdomain in linearSystems)
            //    {
            //        u.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
            //        du.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
            //        rhs.Add(subdomain.ID, subdomain.RHS);
            //    }

            //    double error = 1;

            //    int internalIteration = 0;

            //    Dictionary<int, Vector> du1 = new Dictionary<int, Vector>();
            //    double dlambda1;
            //    if (cycle == 1)
            //    {
            //        BuildMatrices();
            //        provider.ProcessInternalRHS();
            //        provider.
            //        // du1 = CalculateInternalRHS();
            //        // gia na lusw ka8e fora to diaforetiko susthma allazw mesa ston solve.Solve ta RHS me auto pou 8elw px residual, RHS klp klp
                    
            //        // K\lambda*RHS
            //        dlambda1 = 1;
            //    }
            //    else
            //    {
            //        foreach (ILinearSystem subdomain in linearSystems)
            //            du1.Add(subdomain.ID, factor*previousDu[subdomain.ID]);
            //        dlambda1 = factor * previousDLambda;
            //        lambda += dlambda1;
            //    }
                
                                
            //    foreach (ILinearSystem subdomain in linearSystems)
            //    {
            //        u[subdomain.ID] = du1[subdomain.ID];
            //        previousDu[subdomain.ID] = du1[subdomain.ID];
            //    }

            //    double dlambda = dlambda1;
            //    // du1 = CalculateInternalRHS();
            //    // K\lambda*RHS

            //    Vector residual = lambda * rhs - CalculateInternalRHS;

            //    while (error> tolerance)
            //    {
            //        internalIteration++;

            //        // d1 = K\fhat;
            //        // d2 = K\residual;


            //        double ddlambda = -(du1 * d2) / (du1 * d1);
            //        Vector dda = ddlambda * d1 + d2;
                    
            //    }


            //    foreach (ILinearSystem subdomain in linearSystems)
            //    {
            //        du[subdomain.ID] = du[subdomain.ID]+dda[subdomain.ID];
            //        previousDu[subdomain.ID] = du1[subdomain.ID];
            //    }

            //    // du1 = CalculateInternalRHS();
            //    // K\lambda*RHS
            //    residual = lambda * rhs - fint;

            //    error = residual.Norm / (lambda * rhs).Norm;

            //    +++ print Iteration


            //    if (internalIteration == maximumIterations)
            //        throw new OperationCanceledException("Internal Newton-Raphson iterations did not converge!");









            ///}
        }

        #endregion

        #region private methods
        private void UpdateRHS(int step)
        {
            foreach (ILinearSystem subdomain in linearSystems)
            {
                Vector subdomainRHS = ((Vector)subdomain.RHS);
                rhs[subdomain.ID].CopyTo(subdomainRHS.Data, 0);
                subdomainRHS.Multiply(step + 1);
            }
        }

        private void InitializeInternalVectors()
        {
            globalRHS.Clear();
            rhs.Clear();
            u.Clear();
            du.Clear();
            uPlusdu.Clear();

            foreach (ILinearSystem subdomain in linearSystems)
            {
                Vector r = new Vector(subdomain.RHS.Length);
                ((Vector)subdomain.RHS).CopyTo(r.Data, 0);
                r.Multiply(1 / (double)increments);
                rhs.Add(subdomain.ID, r);
                u.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
                du.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
                uPlusdu.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
                previousDu.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
                mappings[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == subdomain.ID).Index].SubdomainToGlobalVector(((Vector)subdomain.RHS).Data, globalRHS.Data);
            }
            rhsNorm = provider.RHSNorm(globalRHS.Data);
        }
        
        private void UpdateInternalVectors()
        {
            globalRHS.Clear();
            foreach (ILinearSystem subdomain in linearSystems)
            {
                Vector r = new Vector(subdomain.RHS.Length);
                ((Vector)subdomain.RHS).CopyTo(r.Data, 0);
                r.Multiply(1 / (double)increments);
                rhs[subdomain.ID] = r;
                mappings[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == subdomain.ID).Index].SubdomainToGlobalVector(((Vector)subdomain.RHS).Data, globalRHS.Data);
            }
            rhsNorm = provider.RHSNorm(globalRHS.Data);
        }
        
        private double CalculateInternalRHS(int currentIncrement, int step)
        {
            globalRHS.Clear();
            foreach (ILinearSystem subdomain in linearSystems)
            {
                if (currentIncrement == 0 && step == 0)
                {
                    Array.Clear(du[subdomain.ID].Data, 0, du[subdomain.ID].Length);
                    Array.Clear(uPlusdu[subdomain.ID].Data, 0, uPlusdu[subdomain.ID].Length);
                    du[subdomain.ID].Add(((Vector)subdomain.Solution));
                    uPlusdu[subdomain.ID].Add(((Vector)subdomain.Solution));
                    du[subdomain.ID].Subtract(u[subdomain.ID]);
                }
                else
                {
                    du[subdomain.ID].Add(((Vector)subdomain.Solution));
                    Array.Clear(uPlusdu[subdomain.ID].Data, 0, uPlusdu[subdomain.ID].Length);
                    uPlusdu[subdomain.ID].Add(u[subdomain.ID]);
                    uPlusdu[subdomain.ID].Add(du[subdomain.ID]);
                }
                Vector internalRHS = (Vector)subdomainUpdaters[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == subdomain.ID).Index].GetRHSFromSolution(uPlusdu[subdomain.ID], du[subdomain.ID]);
                provider.ProcessInternalRHS(subdomain.ID, internalRHS.Data, uPlusdu[subdomain.ID].Data);

                if (parentAnalyzer != null)
                    internalRHS.Add(new Vector(parentAnalyzer.GetOtherRHSComponents(subdomain.ID, uPlusdu[subdomain.ID])));

                Vector subdomainRHS = ((Vector)subdomain.RHS);
                subdomainRHS.Clear();
                for (int j = 0; j <= currentIncrement; j++) subdomainRHS.Add((Vector)rhs[subdomain.ID]);
                subdomainRHS.Subtract(internalRHS);
                mappings[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == subdomain.ID).Index].SubdomainToGlobalVector(subdomainRHS.Data, globalRHS.Data);
            }
            double providerRHSNorm = provider.RHSNorm(globalRHS.Data);
            return providerRHSNorm;
        }
        
        private void ClearIncrementalSolutionVector()
        {
            foreach (ILinearSystem subdomain in linearSystems)
                du[subdomain.ID].Clear();
        }
        
        private void SplitResidualForcesToSubdomains()
        {
            foreach (ILinearSystem subdomain in linearSystems)
            {
                Vector subdomainRHS = ((Vector)subdomain.RHS);
                subdomainRHS.Clear();
                mappings[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == subdomain.ID).Index].SplitGlobalVectorToSubdomain(globalRHS.Data, subdomainRHS.Data);
            }
        }

        private void SaveMaterialStateAndUpdateSolution()
        {
            foreach (ILinearSystem subdomain in linearSystems)
            {
                subdomainUpdaters[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == subdomain.ID).Index].UpdateState();
                u[subdomain.ID].Add(du[subdomain.ID]);
            }
        }
        #endregion
    }
}
