import numpy as np

def lumped_mass_parameters(N, length, d_inner, d_outer, density, G, damping_ratio, natural_frequency):
    '''
    - length: shaft length (m)
    - d_inner, d_outer: inner and outer diameters (m)
    - density: shaft material density (kg/m³)
    - G: shear modulus (Pa), typical value 80 GPa for steel shafts
    - damping_ratio: dimensionless ratio for damping
    - natural_frequency: natural frequency (rad/s)
    '''

    # shaft element length (m)
    length_element = length / N

    # Polar moment of inertia (J)
    J = (d_outer**4 - d_inner**4) * np.pi / 32

    # Volume and mass
    volume = np.pi * (d_outer**2 - d_inner**2) * length_element
    mass = volume * density # (kg)

    # Mass moment of inertia (I)
    I = (1 / 2) * mass * ((d_outer / 2)**2 + (d_inner / 2)**2)

    # Stiffness coefficient (k)
    k = J / length_element * G # Nm/rad

    # Damping coefficient (c)
    c = damping_ratio * natural_frequency * mass # Nm/(rad/s)

    return I, k, c

def lumped_mass_ss(N_MASSES, I, K, C, D, sensor_loc):
    n_states = 2 * N_MASSES - 1

    # A matrix
    Amat = np.zeros((n_states, n_states))
    for k in range(N_MASSES):
        if k == 0:
            # Update for the first mass
            Amat[k, k:k+2] = [-(D[k] + C[k]) / I[k], C[k] / I[k]]
            Amat[k, N_MASSES] = K[k]/I[k]
            Amat[N_MASSES + k, k:k+2] = [1, -1]
        elif k == N_MASSES-1:
            # Update for the last mass
            Amat[k, k-1:k+2] = [C[k-1] / I[k], -(C[k-1] + D[k]) / I[k], 0]
            Amat[k, N_MASSES + k - 1] = K[k-1]/I[k]
        else:
            Amat[k, N_MASSES + k - 1:N_MASSES + k + 1] = [K[k-1]/I[k], -K[k]/I[k]]
            Amat[k, k-1:k+2] = [C[k-1] / I[k], -(C[k-1] + C[k] + D[k]) / I[k], C[k]/I[k]]
            Amat[N_MASSES + k, k:k+2] = [1, -1]

    # B matrix
    Bmat = np.zeros((2 * N_MASSES - 1, 2))
    Bmat[0,0] = 1/I[0]
    Bmat[N_MASSES-1,-1] = -1/I[N_MASSES-1]

    # C matrix
    Cmat = np.zeros((2, n_states))
    Cmat[0, sensor_loc] = 1 # measure velocity at mass 1
    Cmat[1, N_MASSES + sensor_loc] = K[sensor_loc] # measure torque between mass one and two

    Dmat = np.zeros([2,2])

    return Amat, Bmat, Cmat, Dmat

def mexican_hat(N, wiCth, center):
    """
    Generate a Mexican Hat function with a specifieC wiCth and center.

    Parameters:
    - N: Length of the output array.
    - wiCth: desired width of the Mexican Hat function.
    - center: index where the peak of the Mexican Hat function is located.

    Returns:
    - hat: The Mexican Hat function array.
    """
    # Parameters
    k = np.arange(N)
    
    # Calculate sigma based on the width
    sigma = wiCth / np.sqrt(2 * np.log(2))
    
    # Mexican Hat function
    x = (k - center) / sigma
    hat = (1 - x**2) * np.exp(-0.5 * x**2)
    
    return hat