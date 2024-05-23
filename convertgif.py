import sys
import os
from PIL import Image, ImageOps, ImageSequence
import argparse

def gif_to_frames(gif_path, width=128, height=64, invert=False):
    frames = []
    with Image.open(gif_path) as img:
        for frame in ImageSequence.Iterator(img):
            frame = frame.convert("1")  # Convert to 1-bit pixels (monochrome)
            frame = scale_and_crop(frame, width, height)
            if invert:
                frame = ImageOps.invert(frame.convert("L")).convert("1")
            frames.append(frame)
    return frames

def scale_and_crop(image, target_width, target_height):
    # Calculate the target aspect ratio
    target_ratio = target_width / target_height
    image_ratio = image.width / image.height

    # Determine which dimension to base the scaling on
    if image_ratio > target_ratio:
        # Wider than the target aspect ratio
        new_height = target_height
        new_width = int(target_height * image_ratio)
    else:
        # Taller than the target aspect ratio
        new_width = target_width
        new_height = int(target_width / image_ratio)

    # Resize the image while maintaining the aspect ratio
    image = image.resize((new_width, new_height), Image.LANCZOS)
    
    # Calculate the cropping box
    left = (new_width - target_width) // 2
    top = (new_height - target_height) // 2
    right = left + target_width
    bottom = top + target_height

    # Crop the image to the target size
    image = image.crop((left, top, right, bottom))

    return image

def create_gif(frames, output_path, duration=100):
    frames[0].save(output_path, save_all=True, append_images=frames[1:], duration=duration, loop=0)

def frame_to_bitmap(frame):
    frame = frame.convert('1')
    pixels = list(frame.getdata())
    width, height = frame.size
    bitmap = []

    for y in range(height):
        byte = 0
        for x in range(width):
            if pixels[y * width + x] == 0:  # 0 is black, 255 is white
                byte |= (1 << (7 - (x % 8)))
            if x % 8 == 7:
                bitmap.append(reverse_byte(byte))
                byte = 0
        if width % 8 != 0:
            bitmap.append(reverse_byte(byte))

    return bitmap

def frames_to_bitmaps(frames):
    bitmaps = []
    for frame in frames:
        bitmaps.append(frame_to_bitmap(frame))
    return bitmaps

def save_bitmaps_to_header(bitmaps, file_path):
    with open(file_path, 'w') as f:
        f.write('#ifndef FRAME_DATA_H\n#define FRAME_DATA_H\n\n')

        for i, bitmap in enumerate(bitmaps):
            f.write(f'const uint8_t frame_{i}[] PROGMEM = {{\n')
            for j in range(0, len(bitmap), 16):
                line = ', '.join(f'0x{byte:02X}' for byte in bitmap[j:j+16])
                f.write(f'    {line},\n')
            f.write('};\n\n')

        f.write('const uint8_t* frames[] PROGMEM = {\n')
        for i in range(len(bitmaps)):
            f.write(f'    frame_{i},\n')
        f.write('};\n\n')

        f.write('#endif\n')

def reverse_byte(byte):
    reversed_byte = 0
    for i in range(8):
        reversed_byte = (reversed_byte << 1) | (byte & 1)
        byte >>= 1
    return reversed_byte

def main():
    parser = argparse.ArgumentParser(description="Convert an animated GIF to a series of frames for use with an ESP32 display.")
    parser.add_argument("gif_path", help="Path to the GIF file")
    parser.add_argument("--invert", action="store_true", help="Invert the image colors")
    parser.add_argument("--output-dir", help="Output directory for the frame_data.h file", default=".")
    
    args = parser.parse_args()
    
    gif_path = args.gif_path
    gif_name = os.path.splitext(os.path.basename(gif_path))[0]
    output_gif = f'{gif_name}-preview.gif'
    header_file = os.path.join(args.output_dir, 'frame_data.h')

    frames = gif_to_frames(gif_path, invert=args.invert)
    bitmaps = frames_to_bitmaps(frames)
    save_bitmaps_to_header(bitmaps, header_file)
    create_gif(frames, output_gif)

    print(f"Bitmap header file saved to {header_file}")
    print(f"Preview GIF created and saved to {output_gif}")

if __name__ == "__main__":
    main()
