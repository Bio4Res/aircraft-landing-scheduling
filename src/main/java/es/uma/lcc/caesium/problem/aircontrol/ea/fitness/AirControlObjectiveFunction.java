package es.uma.lcc.caesium.problem.aircontrol.ea.fitness;

import java.util.List;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;

/**
 * Interface for objective functions of the Air Control problem
 * @author ccottap
 * @version 1.0
 */
public interface AirControlObjectiveFunction {
	/**
	 * Returns the problem instance
	 * @return the problem instance
	 */
	public AirControlProblem getProblemData();
	
	/**
	 * Returns the landing information encoded in a genotype
	 * @param g the genotype
	 * @return a list of Landing information records
	 */
	public List<LandingInformation> decode (Genotype g);
}
