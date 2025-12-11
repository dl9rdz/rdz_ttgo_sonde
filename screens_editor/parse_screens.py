import traceback
from objects import object_classes

class FileParser:
    def __init__(self):
        self.lines = []  # Store the file content as lines in memory
        self.file_path = None  # Track the file path
        self.global_settings = {}  # Persistent settings for each block

    def read_file(self, file_path):
        """
        Reads the entire file into memory.
        :param file_path: Path to the layout file.
        """
        self.file_path = file_path
        with open(file_path, 'r') as file:
            self.lines = file.readlines()
        self._preprocess_file()

    def _preprocess_file(self):
        """
        Preprocess the file to track global settings for each block.
        """
        current_scale = (13, 22)  # Default scale
        current_fonts = [0, 1]  # Default fonts

        current_block = None
        for line in self.lines:
            line = line.strip()

            # Detect new block
            if line.startswith("@"):
                current_block = line[1:]
                self.global_settings[current_block] = {
                    "scale": current_scale,
                    "fonts": current_fonts,
                }

            # Update settings within a block
            elif current_block:
                if line.startswith("scale="):
                    current_scale = tuple(map(float, line.split("=")[1].split(",")))
                    self.global_settings[current_block]["scale"] = current_scale
                elif line.startswith("fonts="):
                    current_fonts = list(map(int, line.split("=")[1].split(",")))
                    self.global_settings[current_block]["fonts"] = current_fonts

    def write_file(self, file_path=None):
        """
        Writes the in-memory content back to the file.
        :param file_path: Path to the layout file (optional, defaults to the original path).
        """
        if file_path is None:
            file_path = self.file_path
        with open(file_path, 'w') as file:
            file.writelines(self.lines)

    def get_block_names(self):
        """
        Retrieves all block names from the in-memory content.
        :return: List of block names.
        """
        block_names = []
        for line in self.lines:
            line = line.strip()
            if line.startswith("@"):
                block_names.append(line[1:])  # Strip the '@' symbol
        return block_names

    def get_block_content(self, block_name):
        """
        Retrieves the content of a specific block from the in-memory content.
        :param block_name: Name of the block to retrieve.
        :return: The content of the block as a string.
        """
        block_content = []
        capture = False
        for line in self.lines:
            if line.strip() == f"@{block_name}":
                capture = True
            elif line.startswith("@") and capture:
                break
            elif capture:
                block_content.append(line)
        return "".join(block_content)

    def replace_block_content(self, block_name, new_content):
        """
        Replaces the content of a specific block in the in-memory content,
        ensuring the block name line is not duplicated.
        :param block_name: Name of the block to replace.
        :param new_content: New content for the block (list of lines).
        """
        start_index = None
        end_index = None
        for i, line in enumerate(self.lines):
            if line.strip() == f"@{block_name}":
                start_index = i
            elif line.startswith("@") and start_index is not None:
                end_index = i
                break

        if start_index is not None:
            # Replace the block content
            if end_index is None:
                # End of the file
                self.lines = self.lines[:start_index + 1] + new_content
            else:
                self.lines = self.lines[:start_index + 1] + new_content + self.lines[end_index:]
        else:
            # Block not found, add at the end
            self.lines.append(f"@{block_name}\n")
            self.lines.extend(new_content)

    def parse_objects(self, block_name):
        """
        Parses objects from a specific block in the in-memory content.
        :param block_name: Name of the block to parse.
        :return: A list of instantiated objects.
        """
        if block_name in self.global_settings:
            # Get the persistent settings for this block
            scale_y, scale_x = self.global_settings[block_name]["scale"]
            current_font = self.global_settings[block_name]["fonts"]
        else:
            scale_y, scale_x = 13, 22
            current_font = [0, 1]

        objects = []
        current_fg = (1, 1, 1)
        current_bg = None

        content = self.get_block_content(block_name)
        for line in content.splitlines():
          try:
            line = line.strip()

            # Ignore comments and empty lines
            if not line or line.startswith("#"):
                continue

            # Handle scaling
            if line.startswith("scale="):
                scale_y, scale_x = map(float, line.split("=")[1].split(","))
                continue

            # Ignore specific lines
            if line.startswith(("timer", "key", "timeaction")):
                continue

            # Handle fonts
            if line.startswith("fonts="):
                current_font = list(map(int, line.split("=")[1].split(",")))
                continue

            # Handle colors
            if line.startswith("color="):
                color = line.split("=")[1]
                current_fg = (
                    int(color[0:2], 16) / 255,
                    int(color[2:4], 16) / 255,
                    int(color[4:6], 16) / 255,
                )
                current_bg = None if len(color) <= 6 else (
                    int(color[7:9], 16) / 255,
                    int(color[9:11], 16) / 255,
                    int(color[11:13], 16) / 255,
                )
                continue

            # Parse object definitions
            if "=" in line:
                position, definition = line.split("=", 1)
                pos = list(map(float, position.split(",")))

                # Handle x, y, (optional) w
                y = pos[0] 
                x = pos[1]
                w = pos[2] if len(pos) > 2 else 0

                font = current_font[1] if definition[0].isupper() else current_font[0]
                letter, extra = definition[0], definition[1:]

                # Map object types
                object_mapping = {
                    "X": "TextObject",
                    "L": "LatitudeObject",
                    "O": "LongitudeObject",
                    "A": "AltitudeObject",
                    "H": "HSObject",
                    "V": "VSObject",
                    "I": "IDObject",
                    "C": "AFCObject",
                    "T": "TypeObject",
                    "R": "RSSIObject",
                    "F": "FreqObject",
                    "S": "SiteObject",
                    "N": "IPObject",
                    "Z": "VersionObject",
                }

                obj_class_name = object_mapping.get(letter.upper())
                if not obj_class_name:
                    print(f"Unknown object type: {letter}")
                    continue

                # Dynamically instantiate the object
                obj_class = object_classes[obj_class_name]
                obj = obj_class(
                    font=font,
                    x=x,
                    y=y,
                    xscale=scale_x,
                    yscale=scale_y,
                    width=w,
                    extra=extra,
                    fg=current_fg,
                    bg=current_bg,
                )
                objects.append(obj)
          except Exception as e:
            print("Could not parse line", line)
            traceback.print_exc()

        return objects

