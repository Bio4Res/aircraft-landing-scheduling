package es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding;

import java.util.List;

import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.initialization.InitializationOperator;
import es.uma.lcc.caesium.grasp.base.ReactiveGRASP;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlPenaltyObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.grasp.AirControlGRASPObjectiveFunction;

/**
 * Creates a feasible solution for the air control problem using GRASP
 * @author ccottap
 * @version 1.0
 *
 */
public class GRASPInitialization extends InitializationOperator {
	/**
	 * decoder
	 */
	private ReactiveGRASP myRG;
	/**
	 * seed for the GRASP algorithm
	 */
	private long seed4GRASP;
	/**
	 * number of neighbors for GRASP local search
	 */
	private int numNeighbors;
	/**
	 * the objective function for GRASP
	 */
	private AirControlGRASPObjectiveFunction gof;
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;
	
	/**
	 * Generates the operator
	 * @param pars parameters (prob=1.0, seed for GRASP, numNeighbors for GRASP local search -0 if no local search-)
	 */
	public GRASPInitialization(List<String> pars) {
		super(pars);
			
		seed4GRASP = (pars.size()>1) ? Long.parseLong(pars.get(1)) : 1;
		numNeighbors = (pars.size()>2) ? Integer.parseInt(pars.get(2)) : 0;

		myRG = new ReactiveGRASP();
		myRG.setSeed(seed4GRASP);
		myRG.setNumIters(1);					// one solution
		myRG.setIterUpdate(Integer.MAX_VALUE); 	// no update
		myRG.addValue(1);						// greedy or second-best greedy
		gof = null;
		myRG.setObjectiveFunction(gof);
	}
	
	
	@Override
	public void setObjectiveFunction (ObjectiveFunction obj) {
		super.setObjectiveFunction(obj);
		acp = ((AirControlObjectiveFunction)obj).getProblemData();		
		gof = new AirControlGRASPObjectiveFunction(acp);
		gof.setNumNeighbors(numNeighbors);
		myRG.setObjectiveFunction(gof);
	}

	@Override
	protected Individual _apply(List<Individual> parents) {
		myRG.run();
		@SuppressWarnings("unchecked")
		List<LandingInformation> info = (List<LandingInformation>)myRG.getStatistics().getBest(0);
		myRG.getStatistics().clear();
		
		obj.addExtraCost(gof.equivalentCost());
		
		AirControlPenaltyObjectiveFunction p = (AirControlPenaltyObjectiveFunction)obj;
		Individual ind = new Individual();
		ind.setGenome(p.encode(info));
		return ind;
	}

	@Override
	public String toString() {
		return "GRASPInitialization(" + numNeighbors + ")";
	}

}
