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

def second_difference_matrix(n, m):
    '''
    n: number of data points
    m: number of inputs to system
    '''
    D2 = np.eye(n*m) - 2*np.eye(n*m, k=2) + np.eye(n*m, k=4)
    # delete incomplete rows
    D2 = D2[:-2*m, :]

    return D2

def get_sparsity(matrix):
    """
    Finds the indices of non-zero elements in a given matrix.

    Parameters:
    ----------
    matrix : numpy.ndarray
        A 2D numpy array (dense representation) where non-zero elements are to be identified.

    Returns:
    -------
    list of tuple
        A list of tuples where each tuple represents the (row, column) index of a non-zero element.
    
    Example:
    -------
    >>> matrix = np.array([[1, 2, 0],
    ...                    [0, 3, 0]])
    >>> find_nonzero_indices(matrix)
    [(0, 0), (0, 1), (1, 1)]
    """
    # Get the indices of non-zero elements
    rows, cols = np.nonzero(matrix)
    
    # Combine row and column indices into a list of tuples
    nonzero_indices = list(zip(rows, cols))
    
    return nonzero_indices