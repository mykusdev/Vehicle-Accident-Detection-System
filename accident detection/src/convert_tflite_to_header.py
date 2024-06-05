import os

def convert_tflite_to_header(tflite_file_path, header_file_path, array_name):
    if not os.path.isfile(tflite_file_path):
        print(f"Error: The file {tflite_file_path} does not exist.")
        return
    
    with open(tflite_file_path, 'rb') as file:
        tflite_model = file.read()

    # Start writing the header file
    with open(header_file_path, 'w') as header_file:
        header_file.write('#ifndef MODEL_TFLITE_H\n')
        header_file.write('#define MODEL_TFLITE_H\n\n')
        header_file.write(f'const unsigned char {array_name}[] = {{\n')

        # Write the byte array in C format
        for i, byte in enumerate(tflite_model):
            if i % 12 == 0:
                header_file.write('\n    ')
            header_file.write(f'0x{byte:02x}, ')

        # Close the array and add array length
        header_file.write('\n};\n\n')
        header_file.write(f'const unsigned int {array_name}_len = {len(tflite_model)};\n\n')
        header_file.write('#endif  // MODEL_TFLITE_H\n')


# Example usage
tflite_file_path = 'c:/Users/mukes/OneDrive/Desktop/new esp32/src/model.tflite'  # Update with the correct path
header_file_path = 'c:/Users/mukes/OneDrive/Desktop/new esp32/src/model.tflite.h'  # Update with the correct path
array_name = 'model_tflite'
convert_tflite_to_header(tflite_file_path, header_file_path, array_name)
