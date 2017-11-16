using ISAAR.MSolve.IGA.Entities;
using ISAAR.MSolve.Logging.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ISAAR.MSolve.IGA.Logging
{
    public class LinearAnalyzerLogFactory : ILinearAnalyzerLogFactory
    {
        private readonly int[] dofs;
        private readonly Element[] stressElements, forceElements;

        public LinearAnalyzerLogFactory(int[] dofs, Element[] stressElements, Element[] dofElements)
        {
            this.dofs = dofs;
            this.stressElements = stressElements;
            this.forceElements = dofElements;
        }

        public LinearAnalyzerLogFactory(int[] dofs) : this(dofs, new Element[0], new Element[0])
        {
        }

        public IAnalyzerLog[] CreateLogs()
        {
            var l = new List<IAnalyzerLog>();
            l.Add(new DOFsLog(dofs));
            if (stressElements.Length > 0)
                l.Add(new StressesLog(stressElements));
            if (forceElements.Length > 0)
                l.Add(new ForcesLog(forceElements));
            return l.ToArray();
        }
    }
}
