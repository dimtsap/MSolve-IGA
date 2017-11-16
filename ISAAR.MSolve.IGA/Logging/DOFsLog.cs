using ISAAR.MSolve.Logging.Interfaces;
using ISAAR.MSolve.Numerical.LinearAlgebra.Interfaces;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace ISAAR.MSolve.IGA.Logging
{
    public class DOFsLog : IAnalyzerLog
    {
        private readonly int[] dofs;
        private readonly Dictionary<int, double> dofValues = new Dictionary<int, double>();

        public DOFsLog(int[] dofs)
        {
            this.dofs = dofs;
        }

        public Dictionary<int,double> DOFValues { get { return dofValues; } }
        public DateTime StartTime { get; set; }
        public DateTime EndTime { get; set; }

        public override string ToString()
        {
            StringBuilder s = new StringBuilder();
            foreach (int dof in dofValues.Keys)
                s.Append(String.Format("({0}): {1:e}; ", dof, dofValues[dof]));
            return s.ToString();
        }

        #region IResultStorage Members

        public void StoreResults(DateTime startTime, DateTime endTime, IVector solution)
        {
            StartTime = startTime;
            EndTime = endTime;
            foreach (int dof in dofs)
            {
                dofValues[dof] = solution[dof];
                Debug.WriteLine(solution[dof]);
            }
        }

        #endregion
    }
}
