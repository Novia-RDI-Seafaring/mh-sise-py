import numpy as np

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