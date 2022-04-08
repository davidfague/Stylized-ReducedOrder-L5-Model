# Stylized Reduced-Order L5 Model
 
Projects:

(1)	Synaptic input:

(1.1) deliver a random Poisson distribution of stim to each synapse
	Note: 

		Made the following changes:

			create a list of stimulus obj
			assign each stimulus obj to one synapse
			stim.noise=1 completely random event occurence, stim.noise=0 regular intervals

(1.2) distribute synapses at random locations on the dendrites
synapse densities for exc(2.16/um)*200um= 432 and inhib(.22/um)*200um= 44 
	Note: 
		Made the following changes:
			location:
			random number uniform distribution from 0-1
			location parameter operates on 0-1 as a proportion of the total length
			
			Synapse Types:
			randomly assigned synapse types at each location accounting for synapse type densities


(1.3) random seeding to keep the random set of values consistent between simulations
	Note:
		Made the following changes:
			numpy.random.seed


(1.4) lognormal distribution for gmax values
	Note:
	
		From Ben's code:
		def lognormal(m, s):
      		  mean = np.log(m) - 0.5 * np.log((s/m)**2+1)
      		  std = np.sqrt(np.log((s/m)**2 + 1))
      		  #import pdb; pdb.set_trace()
      		  return max(np.random.lognormal(mean, std, 1), 0.00000001)





