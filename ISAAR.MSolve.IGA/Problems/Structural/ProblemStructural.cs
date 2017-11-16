using ISAAR.MSolve.Analyzers.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ISAAR.MSolve.Numerical.LinearAlgebra.Interfaces;
using ISAAR.MSolve.IGA.Entities;
using ISAAR.MSolve.Numerical.LinearAlgebra;
using ISAAR.MSolve.IGA.Problems.Structural.Providers;
using ISAAR.MSolve.Solvers.Interfaces;

namespace ISAAR.MSolve.IGA.Problems.Structural
{
    public class ProblemStructural : IStaticProvider, IImplicitIntegrationProvider,  INonLinearProvider
    {
        Dictionary<int, IMatrix2D> ms, cs, ks;
        private readonly Model model;
        private ElementStructuralStiffnessProvider stiffnessProvider = new ElementStructuralStiffnessProvider();
        private ElementStructuralMassProvider massProvider = new ElementStructuralMassProvider();

        public ProblemStructural(Model model)
        {
            this.model = model;
        }

        private IDictionary<int, IMatrix2D> Ms
        {
            get
            {
                if (ms == null) BuildMs();
                return ms;
            }
        }

        private IDictionary<int, IMatrix2D> Cs
        {
            get
            {
                if (cs == null) BuildCs();
                return cs;
            }
        }

        private IDictionary<int, IMatrix2D> Ks
        {
            get
            {
                if (ks == null)
                    BuildKs();
                else
                    RebuildKs();
                return ks;
            }
        }

        private void BuildKs()
        {
            ks = new Dictionary<int, IMatrix2D>(model.PatchesDictionary.Count);
            ElementStructuralStiffnessProvider s = new ElementStructuralStiffnessProvider();

            int procs = VectorExtensions.AffinityCount;
            var k = model.PatchesDictionary.Keys.Select(x => x).ToArray<int>();
            var internalKs = new Dictionary<int, IMatrix2D>[procs];
            Parallel.ForEach(k.PartitionLimits(procs), limit =>
            {
                if (limit.Item3 - limit.Item2 > 0)
                {
                    internalKs[limit.Item1] = new Dictionary<int, IMatrix2D>(limit.Item3 - limit.Item2);
                    for (int i = limit.Item2; i < limit.Item3; i++)
                        internalKs[limit.Item1].Add(k[i], GlobalMatrixAssemblerSkyline.CalculateGlobalMatrix(model.PatchesDictionary[k[i]], s));
                }
                else
                    internalKs[limit.Item1] = new Dictionary<int, IMatrix2D>();
            });
            for (int i = 0; i < procs; i++)
                foreach (int key in internalKs[i].Keys)
                    ks.Add(key, internalKs[i][key]);
        }

        private void RebuildKs()
        {
            foreach (Patch patch in model.PatchesDictionary.Values)
            {
                if (patch.MaterialsModified)
                {
                    ks[patch.ID] = GlobalMatrixAssemblerSkyline.CalculateGlobalMatrix(patch, stiffnessProvider);
                    patch.ResetMaterialsModifiedProperty();
                }
            }
        }

        private void BuildMs()
        {
            throw new NotImplementedException();
        }

        private void BuildCs()
        {
            throw new NotImplementedException();
        }


        #region IStaticProvider
        public void CalculateMatrix(int id)
        {
            if (ks == null)
                BuildKs();
        }

        public void CalculateMatrix(ILinearSystem patch)
        {
            if (ks == null)
                BuildKs();
            patch.Matrix = this.ks[patch.ID];
        }
        #endregion

        #region IImplicitIntegrationProvider
        public void CalculateEffectiveMatrix(int id, IImplicitIntegrationCoefficients coefficients)
        {
            throw new NotImplementedException();
        }       

        public void DampingMatrixVectorProduct(int id, IVector vIn, double[] vOut)
        {
            throw new NotImplementedException();
        }

        public void GetRHSFromHistoryLoad(int timeStep)
        {
            throw new NotImplementedException();
        }

        public void MassMatrixVectorProduct(int id, IVector vIn, double[] vOut)
        {
            throw new NotImplementedException();
        }

        public void ProcessRHS(int id, IImplicitIntegrationCoefficients coefficients)
        {
            throw new NotImplementedException();
        }

        public void Reset()
        {
            foreach (Patch patch in model.PatchesDictionary.Values)
                foreach (var element in patch.ElementsDictionary.Values)
                    element.ElementType.ClearMaterialState();

            cs = null;
            ks = null;
            ms = null;
        }
        #endregion

        #region INonLinearProvider
        public void ProcessInternalRHS(int id, double[] rhs, double[] solution)
        {
            throw new NotImplementedException();
        }

        public double RHSNorm(double[] rhs)
        {
            return (new Vector(rhs)).Norm;
        }
        #endregion
    }
}
