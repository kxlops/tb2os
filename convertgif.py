import sys
import argparse
from PIL import Image, ImageSequence, ImageOps

def gif_to_monochrome_array(gif_path, width=128, height=64, invert=False):
    # Open the GIF file
    gif = Image.open(gif_path)
    
    # Create an array to store the pixel data for each frame
    frames = []
    preview_frames = []  # For creating the preview GIF
    
    # Iterate through each frame in the GIF
    for frame in ImageSequence.Iterator(gif):
        # Resize the frame to fit the OLED display dimensions with high quality
        resized_frame = frame.resize((width, height), Image.BICUBIC)
        
        # Convert the resized frame to grayscale
        grayscale_frame = resized_frame.convert('L')
        
        # Optionally invert the frame
        if invert:
            grayscale_frame = ImageOps.invert(grayscale_frame)
        
        # Convert to monochrome (black and white)
        monochrome_frame = grayscale_frame.point(lambda x: 0 if x < 128 else 255, '1')
        
        # Get the pixel data
        pixels = list(monochrome_frame.getdata())
        
        # Convert the 2D array to a 1D array of bytes
        byte_array = []
        for y in range(height):
            for x in range(0, width, 8):
                byte = 0
                for bit in range(8):
                    if x + bit < width and pixels[y * width + x + bit] == 0:
                        byte |= (1 << (7 - bit))
                byte_array.append(byte)
        
        # Append the byte array to the list of frames
        frames.append(byte_array)
        preview_frames.append(monochrome_frame)
    
    # Save the preview GIF
    preview_gif_path = gif_path.replace(".gif", "-preview.gif")
    preview_frames[0].save(preview_gif_path, save_all=True, append_images=preview_frames[1:], loop=0, duration=gif.info['duration'])
    
    return frames, preview_gif_path

def write_frame_as_c_array(frame, frame_number, file):
    file.write(f"const unsigned char frame{frame_number}[] PROGMEM = {{\n")
    for i in range(0, len(frame), 16):  # Print 16 bytes per line
        file.write("    " + ", ".join(f"0x{byte:02X}" for byte in frame[i:i+16]) + ",\n")
    file.write("};\n")

def main():
    parser = argparse.ArgumentParser(description='Convert GIF to monochrome C arrays for Arduino.')
    parser.add_argument('gif_path', type=str, help='Path to the GIF file.')
    parser.add_argument('--invert', action='store_true', help='Invert the pixels in the GIF.')
    args = parser.parse_args()

    gif_path = args.gif_path
    invert = args.invert

    frames, preview_gif_path = gif_to_monochrome_array(gif_path, invert=invert)
    header_file_path = "frame_data.h"

    with open(header_file_path, 'w') as file:
        file.write("#ifndef FRAME_DATA_H\n")
        file.write("#define FRAME_DATA_H\n\n")
        
        for i, frame in enumerate(frames):
            file.write(f"// Frame {i + 1}\n")
            write_frame_as_c_array(frame, i + 1, file)
            file.write("\n")
        
        file.write("const unsigned char* frames[] PROGMEM = {\n")
        for i in range(len(frames)):
            file.write(f"  frame{i + 1},\n")
        file.write("};\n\n")
        
        file.write("#endif // FRAME_DATA_H\n")
    
    print(f"Header file saved as: {header_file_path}")
    print(f"Preview GIF saved as: {preview_gif_path}")

if __name__ == "__main__":
    main()
