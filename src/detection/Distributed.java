package detection;

import main.Parameters;
import main.Position;
import main.State;
import main.UAV;
import patterns.SearchPattern;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.*;
import java.util.stream.Collectors;

public class Distributed extends SolverManager {
	
	// constructor with the list of search patterns
	Distributed(List<SearchPattern> candidate){
        super(candidate, State.getUAVs());
    }

    // constructor with the list of the search patterns and uavs
	Distributed(List<SearchPattern> candidate, List<UAV> uavs) {
        super(candidate, uavs);
    }
    
    //definition of class Node. Each node is a search pattern and a time window. Neighbours are search patterns that are possibly in contrast. 
    // Action is the time at which the search pattern is to be initiated, and 0 if the search pattern will not be searched. 
    class Node {
    	private int id; // progressive number from 1
    	private SearchPattern searchPattern; 
    	private double initialTime;
    	private double finalTime;
    	private double windowWidth;
    	private double action; 
    	private List<Node> neighbours = new ArrayList<>();
    	private List<Node> precedingNeighbours = new ArrayList<>();
    	private List<Node> followingNeighbours = new ArrayList<>();
    	
    	// constructor
    	Node(int id, SearchPattern sp, double initialTime, double finalTime) {
    		this.id = id;
    		this.searchPattern = sp;
    		this.initialTime = initialTime;
    		this.finalTime = finalTime;
    		this.windowWidth = finalTime - initialTime;
    		this.action = 0; // initialised at "not selected"
    		//System.out.println("Created node " + this.toString() + " from search pattern " + sp.toString() + " of window " + sp.getMinT() + " - " + sp.getMaxT());
    	}
    	
    	// toString(), gives id, window and action
    	public String toString() {
    		return ((Integer)id).toString() + " w: " + this.initialTime + "-" + this.finalTime + " a: " + this.action;
    	}
    	
    	// Method for filling the neighbours list. Accepts a list of nodes and fills edges with the calling node. 
    	public void setNeighbours(List<Node> nodeList) {
    		for (Node n : nodeList) {
    			// adds n to the neighbours of the current node if they are *incompatible* (and avoids self loops)
    			if (this != n && areNodesConflicting(this, n)){
    				//System.out.println("D(" + this.id + ", " + n.id + ") = " + timeBetween(this, n));
    				this.neighbours.add(n);
    				
    				if (n.initialTime <= this.finalTime) // preceding:
    					this.precedingNeighbours.add(n); 
    				else if (n.finalTime >= this.initialTime) // following:
    					this.followingNeighbours.add(n);
    			}
    		}
    		//System.out.println("Neighbours of node " + this.id + ": " + this.neighbours.toString());
    	}
    	
    	public double restrictToWindow(double d) {
    		if (d > this.finalTime)
    			return finalTime;
    		else if (d < this.initialTime)
    			return initialTime;
    		else 
    			return d;
    	}
    	
    	
    	private double restrictToInitialTime(double d) {
			if (d < this.initialTime)
				return initialTime;
			else
				return d;
		}
    	
    	
    	public double inactionPenalty() {
    		if(this.action != 0) return 0;
    		
    		double tau2 = this.getTau2();
    		double tau1 = this.getTau1();
    		return - Double.max(tau2-tau1, 0);
    	} 

		// computes the utility of the current node (sum of the neighbours' penalties and its own's)
    	public double utility() {
    		double overlapPenalty = 0;
    		//System.out.println("Computing utility for node " + this.id + " of action " + this.action);
    		
    		// penalty for overlapping when they are both active
    		if(this.action != 0) {
    			for(Node n : this.precedingNeighbours) {
    				if(n.action != 0) {
    					overlapPenalty += -Double.max(n.action + timeBetween(n, this) - this.action, 0);
    					//System.out.println("Overlap penalty between " + this.id + ", " + n.id + " of action " + this.action + ", " + n.action + ". Timebetween " + timeBetween(n,this) + ", penalty " + (-Double.max(n.action + timeBetween(n, this) - this.action, 0)));
    				}
    			}
    			for(Node n : this.followingNeighbours) {
    				if(n.action != 0) {
    					overlapPenalty += -Double.max(this.action + timeBetween(this, n) - n.action, 0);
    					//System.out.println("Overlap penalty between " + this.id + ", " + n.id + " of action " + this.action + ", " + n.action + ". Timebetween " + timeBetween(this,n) + ", penalty " + (-Double.max(this.action + timeBetween(this, n) - n.action, 0)));
    				}
    			}
    		}
    		//System.out.println("Total overlapping penalty: " + overlapPenalty);
    		
    		// penalty of inactive nodes
    		int inactionPenalty = 0;
    		inactionPenalty += this.inactionPenalty();
    		for(Node n : this.neighbours) {
    			inactionPenalty += n.inactionPenalty();
    		} 
    		//System.out.println("Total inaction penalty: " + inactionPenalty);

    		
			return overlapPenalty + inactionPenalty;
    	}

		// computes tau1
    	private double getTau1() {
    		try {
    			return this.restrictToWindow(this.precedingNeighbours.stream()
        				.filter(n -> n.action != 0).map(n -> n.action + timeBetween(n, this)).mapToDouble(Double::doubleValue).max().getAsDouble());
    		}
			catch(NoSuchElementException e) { // this happens then all preceding neighbours have action 0
				return this.initialTime;
			}
    	}
    	
    	private double getTau1Unrestricted() {
    		try {
    			return this.restrictToInitialTime(this.precedingNeighbours.stream()
        				.filter(n -> n.action != 0).map(n -> n.action + timeBetween(n, this)).mapToDouble(Double::doubleValue).max().getAsDouble());
    		}
			catch(NoSuchElementException e) { // this happens then all preceding neighbours have action 0
				return this.initialTime;
			}			
		}

		// computes tau2
    	private double getTau2() {
    		try {
    			return this.restrictToWindow(this.followingNeighbours.stream()
        				.filter(n -> n.action != 0).map(n -> n.action - timeBetween(this, n)).mapToDouble(Double::doubleValue).min().getAsDouble());
    		}
    		catch(NoSuchElementException e) { // this happens then all following neighbours have action 0
    			return this.finalTime;
    		}
    	}
    	
    	// computes tau1 of this ignoring the effect of node n
		public double getTau1Ignoring(Node node) {
			try {
				return this.restrictToWindow(this.precedingNeighbours.stream()
	    				.filter(n -> (n.action != 0) && (n.id != node.id)).map(n -> n.action + timeBetween(n, this)).mapToDouble(Double::doubleValue).max().getAsDouble());
			}
			catch(NoSuchElementException e) { // this happens then all preceding neighbours have action 0 (possibly with the exception of node)
				return this.initialTime;
			}
    		
    	}

    	// computes tau2 of this ignoring the effect of node n
		public double getTau2Ignoring(Node node) {
			try {
				return this.restrictToWindow(this.followingNeighbours.stream()
	    				.filter(n -> (n.action != 0) && (n.id != node.id)).map(n -> n.action - timeBetween(this, n)).mapToDouble(Double::doubleValue).min().getAsDouble());
			}
			catch(NoSuchElementException e) { // this happens then all following neighbours have action 0 (possibly with the exception of node)
				return this.finalTime;
			}
    	}
    }
    
	double timeBetween(Node n1, Node n2) {
		return n1.searchPattern.getDuration() + Math.ceil(n1.searchPattern.getOrigin().distance(n2.searchPattern.getOrigin())/Parameters.uavVelocity);
	}
    
	// method for checking compatibility between nodes. Two nodes are compatible if and only if no matter their starting times, there is no conflict between them
	boolean areNodesConflicting(Node n1, Node n2) {
		// there are three possible situations:
		// 1. not overlapping window: must be checked
		// 2. one window is subset of another: they are conflicting
		// 3. partially overlapping windows: they are conflicting
		
		// first, order them by who starts first
		Node tmp;
		if (n2.initialTime < n1.initialTime) {
			tmp = n1;
			n1 = n2;
			n2 = tmp;
		}
				
		// situation 2 and 3:
		if (n1.finalTime > n2.initialTime)	
			return true;
		else { // situation 1:
			//System.out.println("D(" + n1.id + ", " + n2.id + ") = " + (n1.searchPattern.getDuration() + Math.ceil(n1.searchPattern.getOrigin().distance(n2.searchPattern.getOrigin())/Parameters.uavVelocity))); 
			return n1.finalTime + timeBetween(n1, n2) > n2.initialTime;
		}
	}
                
	/*
	 * function createGraph(): takes a list of search patterns. Does:
	 * 1. Creates all nodes, duplicating search patterns if necessary
	 * 2. After their creation, puts them in the list nodesList
	 * 3. Spans the list to fill the attribute "neighbours"
	 * 4. Returns the list
	 */
    List<Node> createGraph(List<SearchPattern> candidates){
    	
    	// greedy initialization - produce the greedy schedule
    	HashMap<UAV,ArrayList<SearchPattern>> sequence = new GreedyAlgorithm(candidates).sequence();
		HashMap<SearchPattern, Integer> greedyPlan = new HashMap<>(); // greedy schedule
        for (UAV u : sequence.keySet()){ // there is only one UAV
			List<SearchPattern> planList = sequence.get(u); // search patterns resulting from the greedy schedule
	        int time = State.getTime() + 1;
	        int startTime = 0;
	        SearchPattern previous = null;
	        for (SearchPattern sp : sequence.get(u)){
	            startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
	            greedyPlan.put(sp, startTime);
	            previous = sp;
	        }
        }
        System.out.println("Greedy plan: " + greedyPlan.toString() + "\n");
    	
    	List<Node> nodesList = new ArrayList<>();
    	int id = 0;
    	for (SearchPattern sp : candidates) {
    		System.out.println(sp.toString() + " " + (sp.getMinT()) + "-" + (sp.getMaxT()));   		
    		double numSplits = Math.ceil((sp.getMaxT()-sp.getMinT())/sp.getDuration());
    		double sizeSplits = Math.floor((sp.getMaxT()-sp.getMinT())/numSplits);
    		double ti = sp.getMinT();
    		double tf = ti + sizeSplits;
    		for (int k = 0; k < numSplits; k++) {
    			
    			Node n = new Node(id, sp, ti, tf); 			
        		
        		// greedy initialization
        		if(greedyPlan.containsKey(sp)){
        			if(greedyPlan.get(sp) >= ti && greedyPlan.get(sp) < tf){
        				n.action = greedyPlan.get(sp);
        			}	
        		}
        		
    			nodesList.add(n);
				System.out.println("Node added. Id " + id + " search pattern " + sp.toString() + " (time window " + ti + "-" + tf + ")");
				id += 1;
    			ti = tf;
    			tf = ti + sizeSplits;
    		}
    	}
    	    	
		// filling neighbours
    	for (Node n : nodesList)
    		n.setNeighbours(nodesList);
		
		System.out.println("Created graph. Search patterns: " + candidates.size() + ". Nodes: " + nodesList.size());
		
    	return nodesList;
    }
    
    private ArrayList<SearchPattern> convertToPlan(List<Node> graph) {
		return (ArrayList<SearchPattern>) graph.stream().filter(n -> n.action != 0).sorted(nodeComparator)
				.map(n -> n.searchPattern).collect(Collectors.toList());
	}

	private boolean isFeasible(ArrayList<SearchPattern> newPlanSkeleton) {
		if (newPlanSkeleton.isEmpty())
			return false;
		
        int startTime = 0;
        SearchPattern previous = null;
        UAV u = uavs.get(0); // (assuming 1 uav)

		for (SearchPattern sp : newPlanSkeleton) {
            startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
            if (startTime > sp.getMaxT())
            	return false;
            previous = sp;
		}
		return true;
	}

	// comparator to order nodes (which one begins first)
    public Comparator<Node> nodeComparator = new Comparator<Node>() {         // Node Comparator: which one comes first
        @Override
        public int compare(Node n1, Node n2) {
            return (int) (n1.action - n2.action);
        }
    };
    
    public Comparator<SearchPattern> searchPatternComparator = new Comparator<SearchPattern>() {         // Node Comparator: which one comes first
        @Override
        public int compare(SearchPattern sp1, SearchPattern sp2) {
            return sp1.toString().compareTo(sp2.toString());
        }
    };
        
    // function that for each UAV (so far only one) gives its plan as a list of SearchPattern
    // it's the function that samples!
    public HashMap<UAV, ArrayList<SearchPattern>> sequence() {  
    	
    	double timezero = System.currentTimeMillis(); // saving initial time
        List<Node> graph = createGraph(candidateSearchPattern); // Definition and creation of the graph
        double eta = Parameters.eta;   // Noise parameter
                
        // initialising variables for the iterations 
        ArrayList<SearchPattern> bestPlan = convertToPlan(graph); // best configuration
        double Gmax = evaluate(bestPlan); // best value of the functional
        System.out.println("Starting at plan " + bestPlan.toString() + " of value " + Gmax);
        Node sampledNode;
        double u0;
        double p0;
    	Set<Double> partition = new HashSet<>();
    	double timeBetween;
    	double tt1;
    	double tt2;
    	Double[] a;
    	Double[] b;
    	double[] C;
    	double integral;
    	double pTime;
    	double Z;
        double previousAction;
        double newAction;
        
        // initialising variables for plan evaluation
        double G;
        int minsize;
        int firstDifferent;
        ArrayList<SearchPattern> prevPlanSkeleton = new ArrayList<>();
        ArrayList<SearchPattern> newPlanSkeleton = new ArrayList<>(); 
        double[] PSarray = new double[graph.size()];
        double[] TSarray = new double[graph.size()];
        HashMap<Integer, HashMap<List<Position>, Double>> pDestarray = new HashMap<>();
        double[] pStararray = new double[graph.size()];        
		double PS = 0;
		double TS = 0;
		double pStar;
		double newPS;
        double gamma;
        int nDestination = cityNameMapping.size();
        double pi = 1. / nDestination;	
        int ind;
        double previous;
        double pC;
                
        // register of the plan skeletons
        HashSet<String> register = new HashSet<>();
        String s;
        
        /*
        // for "time progression"
        double timeProgressionStep = 1000; // 1000 ms = 1 s 
        double timeProgressionCheck = System.currentTimeMillis();
    	try {
            Files.write(Paths.get("time_progression.txt"), ("\n" + Gmax + " ").getBytes(), StandardOpenOption.APPEND); // va a caporiga per la nuova istanza
        }catch (IOException e) {
            //exception handling left as an exercise for the reader
        }
    	try {
            Files.write(Paths.get("distinct_plans.txt"), ("\n" + register.size() + " ").getBytes(), StandardOpenOption.APPEND);
        }catch (IOException e) {
            //exception handling left as an exercise for the reader
        }
        */
        
        // Start iterations that update attribute "action" in the nodes
        Random r = new Random(Parameters.seed);
        double elapsed = System.currentTimeMillis() - timezero;
        double timeOut = Parameters.planTimeLimit*1000;
        int iterationsCount = 0;
		while (elapsed < timeOut) {
			
			/*
			if (System.currentTimeMillis() - timeProgressionCheck > timeProgressionStep) {
				timeProgressionCheck = System.currentTimeMillis();
		    	try {
		            Files.write(Paths.get("time_progression.txt"), (Gmax + " ").getBytes(), StandardOpenOption.APPEND);
		        }catch (IOException e) {
		            //exception handling left as an exercise for the reader
		        }
		    	try {
		            Files.write(Paths.get("distinct_plans.txt"), (register.size() + " ").getBytes(), StandardOpenOption.APPEND);
		        }catch (IOException e) {
		            //exception handling left as an exercise for the reader
		        }
			}
			*/
			
        	iterationsCount += 1;
        	
        	//System.out.println("\n\nIteration " + iterationsCount);
        	
        	// sampling
        	sampledNode = graph.get(r.nextInt(graph.size())); // sample random node 
        	previousAction = sampledNode.action; // previous action (to be changed)
        	//previousUtility = sampledNode.utility();
        	/*
        	System.out.println("\nSampled node " + sampledNode.toString() + " of neighbours: ");
        	for (Node n : sampledNode.precedingNeighbours) {
        		System.out.println(n.toString() + ", action " + n.action + " time between: " + timeBetween(n, sampledNode) + " tau2: " + n.getTau2Ignoring(n));
        	}
        	for (Node n : sampledNode.followingNeighbours) {
        		System.out.println(n.toString() + ", action " + n.action + " time between: " + timeBetween(sampledNode, n) + " tau1: " + n.getTau1Ignoring(n));
        	}
        	*/
        	
        	// utility of action = 0
        	sampledNode.action = 0;
        	u0 = sampledNode.utility();
        	//System.out.println("Utility of 0: " + u0);
        	p0 = Parameters.delta*Math.exp(eta*u0);
        	
        	// adding knots to the partition
        	partition.clear();
        	double initialTime = sampledNode.initialTime; // create ogni volta altrimenti bisticcia con stream.filter
        	double finalTime = sampledNode.finalTime;
        	partition.add(initialTime);
        	partition.add(finalTime);
        	for (Node n : sampledNode.precedingNeighbours) {
        		timeBetween = timeBetween(n, sampledNode);
        		if (n.action == 0) partition.add(n.getTau2Ignoring(sampledNode) + timeBetween); // earliest time of departure
        		else partition.add(n.action + timeBetween);
        	}
        	for (Node n : sampledNode.followingNeighbours) {
        		timeBetween = timeBetween(sampledNode, n);
        		if (n.action == 0) partition.add(n.getTau1Ignoring(sampledNode) - timeBetween); // earliest time of arrival
        		else partition.add(n.action - timeBetween);
        	}
        	
        	// filtering out knots and building vector a
        	a = partition.stream().filter(d -> d >= initialTime && d <= finalTime).toArray(Double[]::new);       	
        	Arrays.sort(a);
        	
        	// defining vector b
        	b = new Double[a.length];
        	for (int i = 0; i < a.length; i++) {
        		sampledNode.action = a[i];
        		b[i] = sampledNode.utility(); 
        	}
        	
        	// sampling new action
        	C = getC(a, b, eta);
        	integral = 0;
        	for (int i = 1; i < C.length; i++) {
        		integral += C[i];
        	}
        	pTime = (1-Parameters.delta)*integral/sampledNode.windowWidth;
        	Z = p0 + pTime;
        	
        	//double meanutil = getmeanutil(a, b);
        	//System.out.println("Action 0: u0 = " + u0 + " Math.exp(u0) = " + p0 + " p0 = " + (p0/Z));
        	//System.out.println("Action t: umean = " + meanutil + " exp(eta*meanutil) = " + Math.exp(eta*meanutil) + " integral: " + integral + " integral normalized = " + pTime + " pTime = " + (pTime/Z));
        	
        	if (r.nextDouble()*Z < p0) {
        		newAction = 0;
        		sampledNode.action = newAction;
        	}
        	else {
        		newAction = sample(a, b, C, integral, eta, r);
        		sampledNode.action = newAction;
        	}
        	
        	//System.out.println("New action: " + newAction);
        	//System.out.println("Utility of new action: " + sampledNode.utility());
        	//System.out.println(sampledNode.precedingNeighbours);
        	//System.out.println(sampledNode.followingNeighbours);

        	// if action didn't change, set it back and go to next loop iteration
        	if (newAction == previousAction) 
        		continue;
        	
        	// if action changed, update Psi
        	//newUtility = sampledNode.utility();
    		//Psi = Psi - previousUtility + newUtility;  
    		//System.out.println("Action changed. New utility: " + newUtility + " Psi: " + Psi);
    			
    		newPlanSkeleton = convertToPlan(graph); // extract plan skeleton
    		if (isFeasible(newPlanSkeleton)) {   
	    		
	    		// add to register if new
		    	s = newPlanSkeleton.toString();
		    	if (register.contains(s))
		    		continue;
	    		register.add(s);
	    		//numPlansEvaluated += 1;
    			//System.out.println("Evaluating new plan " + s);
	    		
	    		// START EVALUATION
		    	// extracts index of first different search pattern
    			minsize = Math.min(prevPlanSkeleton.size(), newPlanSkeleton.size());
    			firstDifferent = minsize; // assumed to be one subset of the other. This also over prev = empty
		    	for (int k = 0; k < minsize; k++) {
		    		if (prevPlanSkeleton.get(k) != newPlanSkeleton.get(k)) {
		    			firstDifferent = k;
		    			break;
		    		}
		    	}
	
				// continuing
				for (int k = firstDifferent; k < newPlanSkeleton.size(); k++) {
					// PS
					if (k == 0) {
						newPS = 0;						
					}
					else {
						PS = PSarray[k-1];
						pStar = pStararray[k-1];
						newPS = PS + pStar*(1-PS);
						//System.out.println("PS update: k = " + k + ", PS = " + newPS);
					}
					PSarray[k] = newPS;
										
					// TS
					if (k == 0) {
						TS = 0;
					}
					else {
						TS = TS + (newPlanSkeleton.get(k-1).getMaxT()+newPlanSkeleton.get(k-1).getMinT())*0.5/maxExpectedTime*(newPS - PS);
					}
					TSarray[k] = TS;
					
					// PSdest
					HashMap<List<Position>, Double> toInsert = new HashMap<>();
					if (k == 0) {
						for (List<Position> d : cityNameMapping.keySet()) {
							toInsert.put(d, pi);  // initialised with the uniform a priori
						}
					}
					else {
						gamma = newPlanSkeleton.get(k-1).getGamma(); // gamma_k is needed, but list indexing starts at 0
						for (List<Position> x : cityNameMapping.keySet()) {
							if (newPlanSkeleton.get(k-1).getCompatibleDestinations().contains(x))
								ind = 1;
							else 
								ind = 0;
							
							previous = pDestarray.get(k-1).get(x);
							pStar = pStararray[k-1];
							toInsert.put(x, previous*(1-gamma*ind)/(1-pStar));
						}
					}
					pDestarray.put(k, toInsert);
																									
					// pS 
					pStar = newPlanSkeleton.get(k).getGamma();
					pC = 0;
					for (List<Position> y : newPlanSkeleton.get(k).getCompatibleDestinations()) {
						pC += pDestarray.get(k).get(y);
					}
					pStar *= pC;
					pStararray[k] = pStar;
				}
				
				prevPlanSkeleton = newPlanSkeleton;
		
				// final
				PS = PSarray[newPlanSkeleton.size()-1];
				TS = TSarray[newPlanSkeleton.size()-1];
				pStar = pStararray[newPlanSkeleton.size()-1];
				newPS = PS + pStar*(1-PS);
				TS = TS + (newPlanSkeleton.get(newPlanSkeleton.size()-1).getMaxT() + newPlanSkeleton.get(newPlanSkeleton.size()-1).getMinT())*0.5/maxExpectedTime*(newPS - PS);
				PS = newPS;
				G = PS - Parameters.weight*TS;
				
		    	if (G > Gmax){
		    		Gmax = G;
		    		bestPlan = newPlanSkeleton;		    		            
		    		System.out.println("Plan updated. Iteration: " + iterationsCount + " New plan: " + bestPlan.toString() + " value: " + Gmax);
		    	}
    		}
    		elapsed = System.currentTimeMillis() - timezero;
        }
		
		System.out.println("Iterations: " + iterationsCount + " Distinct plans: " + register.size()); 
        		
        // assigning plan to UAV
    	HashMap<UAV, ArrayList<SearchPattern>> assignments = new HashMap<>();
        assignments.put(uavs.get(0), bestPlan);
        
    	return assignments;
    }

	// calcolo della costante Z di normalizzazione attraverso C
    private double[] getC(Double[] a, Double[] b, double eta) {
    	int n = a.length;
    	double[] C = new double[n];
    	C[0] = 0;
    	double previousExp = Math.exp(eta*b[0]);
    	double newExp;
    	for (int i = 1; i < n; i++) {
    		if (b[i].equals(b[i-1])) { // per evitare divisione per 0
    			C[i] = (a[i]-a[i-1]) * previousExp;
    		}
    		else {
    			newExp = Math.exp(eta*b[i]);
    			C[i] = (a[i]-a[i-1])/(b[i]-b[i-1]) * ( newExp - previousExp )/eta;
    			previousExp = newExp;
    		}
    		//System.out.println("C[" + i + "] = " + C[i]);
    	}
    	return C;
    }
    
    // sampling dalla densita' esponenziale a tratti univocamente definita da a e b
    private double sample(Double[] a, Double[] b, double[] C, double integral, double eta, Random r) {
    	int n = a.length;
    	double y = r.nextDouble();
    	double ybar = integral*y;
    	//System.out.println("Z: " + integral + " y: " + y + " ybar: " + ybar);
    	double Ccum = 0;
    	int i = 0;
    	for (int j = 0; j < n; j++) {
    		if (ybar <= Ccum+C[j]) {
    			i = j;
    			break;
    		}
    		Ccum += C[j];
    	}
    	//System.out.println("Ccum: " + Ccum);
    	double x;
    	if (b[i].equals(b[i-1])) {
    		x = a[i-1] + (ybar - Ccum)*Math.exp(-eta*b[i-1]);
    	}
    	else {
    		x = a[i-1] + (a[i]-a[i-1])/(b[i]-b[i-1]) * ( - b[i-1] + Math.log(eta*(b[i]-b[i-1])/(a[i]-a[i-1])*(ybar - Ccum) + Math.exp(eta*b[i-1]))/eta);
    	}
		return x;
    }

    // method for evaluating T(S)
    private double evaluateTime(List<SearchPattern> planSkeleton) {
    	
    	if (planSkeleton.isEmpty())
    		return 0;
    	
    	// parameters
 		int planLength = planSkeleton.size();
 		 		
 		// initialisation
 		// TS 
 		double TS = 0;

 		// updating
 		for (int k = 1; k < planLength; k++) { // does (planLength-1) loops
 			// PS and TS
 			TS = TS + (planSkeleton.get(k-1).getMinT() + planSkeleton.get(k-1).getMaxT())* 0.5;
 		}
 		
 		// final
 		TS = TS + (planSkeleton.get(planLength-1).getMinT() + planSkeleton.get(planLength-1).getMaxT())*0.5;
 		
 		return TS;
    }
    
	// method for evaluating G(S)
 	private double evaluate(List<SearchPattern> planSkeleton) {
 		if (planSkeleton.isEmpty())
 			return 0;
 		
 		// parameters
 		int planLength = planSkeleton.size();
 		int nDestination = cityNameMapping.size();
 		 		
 		// initialisation
 		// TS 
 		double TS = 0;
 		// PS
 		double PS = 0;
 		// pDest
 		HashMap<List<Position>, Double> pDest = new HashMap<>();
 		double pi = 1.0/nDestination;
 		for (List<Position> d : cityNameMapping.keySet()) {
 			pDest.put(d, pi); // initialised with the uniform a priori
 		}
 		
 		double ind = 0; // initialising for later
 		// pS
 		double pStar = planSkeleton.get(0).getGamma();
 		double pC = 0;
 		for (List<Position> d : planSkeleton.get(0).getCompatibleDestinations()) {
 			pC += pDest.get(d);
 		}
 		pStar *= pC;

 		// updating
 		double gamma;
 		double newPS;
 		for (int k = 1; k < planLength; k++) { // does (planLength-1) loops
 			// PS and TS
 			newPS = PS + pStar*(1-PS);
 			TS = TS + (planSkeleton.get(k-1).getMinT() + planSkeleton.get(k-1).getMaxT())* 0.5/maxExpectedTime*(newPS - PS);
 			PS = newPS;
 			// PSdest
 			gamma = planSkeleton.get(k-1).getGamma(); // gamma_k is needed, but list indexing starts at 0
 			for (List<Position> x : cityNameMapping.keySet()) {
 				if (planSkeleton.get(k-1).getCompatibleDestinations().contains(x))
 					ind = 1;
 				else 
 					ind = 0;
 				pDest.replace(x, pDest.get(x)*(1-gamma*ind)/(1-pStar));
 			}
 			// PSstar 
 			pC = 0;
 			for (List<Position> y : planSkeleton.get(k).getCompatibleDestinations()) {
 				pC += pDest.get(y);
 			}
 			gamma = planSkeleton.get(k).getGamma();
 			pStar = gamma*pC;
 		}
 		
 		// final
 		newPS = PS + pStar*(1-PS);
 		TS = TS + (planSkeleton.get(planLength-1).getMinT() + planSkeleton.get(planLength-1).getMaxT())*0.5/maxExpectedTime*(newPS-PS);
 		PS = newPS;
 		double GS = PS - Parameters.weight*TS;
 		
 		return GS;
 	}
    
    // original version
	double distanceSP(SearchPattern sp1, SearchPattern sp2, UAV u){

        if (sp1 == null){
            return  Math.ceil(u.getCurrent().distance(sp2.getOrigin())/Parameters.uavVelocity);
        }
        return Math.ceil(sp1.getOrigin().distance(sp2.getOrigin())/ Parameters.uavVelocity)+ sp1.getDuration();
    }
    
	// function outputting the result
    public HashMap<String, HashMap<Integer, List<SearchPattern>>> getPlan(){
        HashMap<String,HashMap<Integer,List<SearchPattern>>> timedPlan = new  HashMap<>();
        HashMap<UAV,ArrayList<SearchPattern>> sequence = sequence();
        for (UAV u : sequence.keySet()){
		List<SearchPattern> planList = sequence.get(u);
        	double G = evaluate(sequence.get(u));
			System.out.println("Solver: " + Parameters.solver + "\nSeed: " + Parameters.seed + "\nUAV: " + u.getUavName() + "\nk: " + Parameters.weight + "\nDouble-checking functional value: " + evaluate(planList) + "\nExpectedTime: " + evaluateTime(planList) + "\nPlan: " + planList.toString());
        	try {
                Files.write(Paths.get("results.txt"), (G + "\n").getBytes(), StandardOpenOption.APPEND);
            }catch (IOException e) {
                //exception handling left as an exercise for the reader
            }
        	HashMap<Integer,List<SearchPattern>> plan = new HashMap<>();
            int time = State.getTime() + 1;
            int startTime = 0;
            SearchPattern previous = null;
            for (SearchPattern sp : sequence.get(u)){
                List<SearchPattern> tmpList = new ArrayList<>();
                tmpList.add(sp);
                plan.put(time,tmpList);
                startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
                time = startTime + State.getTime() + new Double(sp.getDuration()).intValue();
                previous = sp;
            }
            timedPlan.put(u.getUavName(),plan);
        }
        return timedPlan;
    }
}
