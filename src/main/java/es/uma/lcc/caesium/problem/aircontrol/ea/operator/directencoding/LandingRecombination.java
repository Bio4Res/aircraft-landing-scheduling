package es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.recombination.RecombinationOperator;
import es.uma.lcc.caesium.ea.util.EAUtil;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.LandingRepair;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlPenaltyObjectiveFunction;

/**
 * Recombines two lists of landing slots by arranging them in landing order, removing duplicates,
 * and adjusting landing times to fulfill constraints
 * @author ccottap
 * @version 1.0
 */
public class LandingRecombination extends RecombinationOperator {
	/**
	 * repairer
	 */
	private LandingRepair lr;
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;
	/**
	 * arity of the operator
	 */
	private int arity;
	
	/**
	 * Creates the operator
	 * @param pars operator parameters: probability of application and arity (optional; 2 by default) 
	 */
	public LandingRecombination(List<String> pars) {
		super(pars);
		arity = (pars.size() > 1) ? Integer.parseInt(pars.get(1)) : 2;
		acp = null;
		lr = new LandingRepair();
		lr.setData(acp);
	}

	
	@Override
	public void setObjectiveFunction (ObjectiveFunction obj) {
		super.setObjectiveFunction(obj);
		acp = ((AirControlObjectiveFunction)obj).getProblemData();
		lr.setData(acp);
	}
	
	
	@Override
	protected Individual _apply(List<Individual> parents) {
		AirControlPenaltyObjectiveFunction p = (AirControlPenaltyObjectiveFunction)obj;
		int numFlights = acp.getNumFlights();
		
		List<List<LandingInformation>> parentalInfo = new ArrayList<List<LandingInformation>> (arity);
		for (Individual ind: parents) {
			parentalInfo.add(p.decode(ind.getGenome()));
		}
		
		int n = numFlights*arity;
		List<LandingInformation> li = new ArrayList<LandingInformation>(n);
		for (int i=0; i<n; i++) {
			int j;
			do {
				j = EAUtil.random(arity);
			} while (parentalInfo.get(j).isEmpty());
			li.add(parentalInfo.get(j).get(0));
			parentalInfo.get(j).remove(0);
		}
		
		li.sort(null);
		
		List<LandingInformation> newInfo = new ArrayList<LandingInformation>(numFlights);
		int cost = lr.repair(li, newInfo);
		
		Genotype g = p.encode(newInfo);
		
		Individual ind = new Individual();
		ind.setGenome(g);
		
		obj.addExtraCost((double)cost/(double)numFlights);

		return ind;
	}



	@Override
	public int getArity() {
		return arity;
	}

	@Override
	public String toString() {
		return "LandingRecombination(" + prob + ", " + arity + ")";
	}

}
