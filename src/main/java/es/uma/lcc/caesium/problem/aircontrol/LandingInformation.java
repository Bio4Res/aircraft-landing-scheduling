package es.uma.lcc.caesium.problem.aircontrol;

/**
 * Landing information for a flight 
 * @param flightID ID of the flight
 * @param time assigned time for landing
 * @param runway assigned runway for landing
 */
public record LandingInformation(String flightID, long time, int runway) implements Comparable <LandingInformation> {
	@Override
	public String toString() {
		return "[" + flightID + ", " + "t=" + time + ", r=" + runway + "]"; 
	}

	@Override
	public int compareTo(LandingInformation other) {
		int cmp = Long.compare(time, other.time);
		if (cmp != 0)
			return cmp;
		else {
			return Integer.compare(runway, other.runway());
		}
	}
}