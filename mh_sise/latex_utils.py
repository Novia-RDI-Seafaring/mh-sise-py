import os

def save_table(template_path, values_dict, output_dir='results', output_filename='filled_table.tex'):
    """
    Fills a LaTeX table template with values from a dictionary and saves the result to a file.

    Parameters:
    - template_path: str, path to the LaTeX table template file.
    - values_dict: dict, dictionary with placeholders as keys and replacement values.
    - output_dir: str, directory where the output file will be saved (default is 'results').
    - output_filename: str, name of the output LaTeX file (default is 'filled_table.tex').

    Raises:
    - FileNotFoundError: If the template file is not found.
    """
    # Check if the template file exists
    if not os.path.exists(template_path):
        raise FileNotFoundError(f"Error: {template_path} not found!")
    
    # Load the LaTeX table template
    with open(template_path, 'r') as file:
        template = file.read()

    # Debugging: Print the template before replacements
    print("Original template:")
    print(template)

    # Replace the placeholders with actual values
    for placeholder, value in values_dict.items():
        placeholder_to_replace = f'<<{placeholder}>>'
        
        if placeholder_to_replace in template:
            if value is None:
                print(f"Placeholder {placeholder_to_replace} found but no value assigned!")
            else:
                print(f"Replacing {placeholder_to_replace} with {value}")
                template = template.replace(placeholder_to_replace, str(value))
        else:
            print(f"Placeholder {placeholder_to_replace} not found in the template!")

    # Debugging: Print the template after replacements
    print("\nTemplate after replacements:")
    print(template)

    # Ensure the output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Save the filled template to a new file
    output_file_path = os.path.join(output_dir, output_filename)
    with open(output_file_path, 'w') as file:
        file.write(template)

    print(f"Table filled and saved to {output_file_path}")