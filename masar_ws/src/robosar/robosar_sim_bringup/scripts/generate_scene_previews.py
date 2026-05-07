#!/usr/bin/env python3

import json
import os
from typing import Dict, List, Tuple

import yaml
from PIL import Image, ImageDraw, ImageFont


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PACKAGE_DIR, "config", "sim_scenes.json")
OUTPUT_DIR = os.path.join(PACKAGE_DIR, "config", "scene_previews")
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

SCENE_ACCENTS = {
    "tepper": (61, 148, 255),
    "robosar_apartment_smallv1": (46, 204, 113),
}

CATEGORY_STYLES = {
    "victims": {
        "color": (42, 90, 255),
        "label": "Victims",
        "prefix": "V",
        "shape": "circle",
    },
    "fire_sources": {
        "color": (255, 92, 25),
        "label": "Fire Sources",
        "prefix": "F",
        "shape": "circle",
    },
    "transport_objects": {
        "color": (255, 196, 0),
        "label": "Transport Objects",
        "prefix": "T",
        "shape": "square",
    },
}


def _load_scene_config() -> Dict:
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _load_map_info(map_yaml_path: str) -> Dict:
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        map_info = yaml.safe_load(f)

    image_path = map_info["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
    map_info["image_path"] = image_path
    return map_info


def _world_to_pixel(
    world_x: float, world_y: float, map_info: Dict, image_height: int
) -> Tuple[int, int]:
    resolution = float(map_info["resolution"])
    origin_x, origin_y = map_info["origin"][0], map_info["origin"][1]
    px = int(round((world_x - origin_x) / resolution))
    py_from_bottom = int(round((world_y - origin_y) / resolution))
    py = image_height - 1 - py_from_bottom
    return px, py


def _draw_marker(
    draw: ImageDraw.ImageDraw,
    x: int,
    y: int,
    style: Dict,
    marker_text: str,
    font: ImageFont.ImageFont,
) -> None:
    color = style["color"]
    outline = (0, 0, 0)
    radius = 7

    if style["shape"] == "square":
        draw.rectangle((x - radius, y - radius, x + radius, y + radius), fill=color, outline=outline, width=2)
    else:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color, outline=outline, width=2)

    text_x = x + 10
    text_y = y - 8
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        draw.text((text_x + dx, text_y + dy), marker_text, fill=(255, 255, 255), font=font)
    draw.text((text_x, text_y), marker_text, fill=outline, font=font)


def _draw_legend(
    draw: ImageDraw.ImageDraw,
    scene_name: str,
    map_name: str,
    accent: Tuple[int, int, int],
    categories: List[str],
    canvas_width: int,
    font: ImageFont.ImageFont,
    title_font: ImageFont.ImageFont,
) -> None:
    legend_x = canvas_width - 330
    legend_y = 60
    draw.rounded_rectangle(
        (legend_x - 18, legend_y - 18, canvas_width - 20, legend_y + 170),
        radius=12,
        fill=(250, 250, 250),
        outline=(40, 40, 40),
        width=2,
    )
    draw.text((legend_x, legend_y - 4), "Scene:", fill=accent, font=title_font)
    draw.text((legend_x, legend_y + 28), scene_name, fill=accent, font=font)
    draw.text((legend_x, legend_y + 56), f"Map: {map_name}", fill=(30, 30, 30), font=font)

    row_y = legend_y + 92
    for category in categories:
        style = CATEGORY_STYLES[category]
        icon_x = legend_x + 8
        icon_y = row_y + 8
        if style["shape"] == "square":
            draw.rectangle((icon_x - 6, icon_y - 6, icon_x + 6, icon_y + 6), fill=style["color"], outline=(0, 0, 0), width=2)
        else:
            draw.ellipse((icon_x - 6, icon_y - 6, icon_x + 6, icon_y + 6), fill=style["color"], outline=(0, 0, 0), width=2)
        draw.text((legend_x + 24, row_y), style["label"], fill=(30, 30, 30), font=font)
        row_y += 28


def _load_font(size: int) -> ImageFont.ImageFont:
    if os.path.isfile(FONT_PATH):
        return ImageFont.truetype(FONT_PATH, size=size)
    return ImageFont.load_default()


def _build_scene_preview(scene_name: str, scene_data: Dict) -> str:
    map_info = _load_map_info(scene_data["map_yaml"])
    base_map = Image.open(map_info["image_path"]).convert("L")
    base_rgb = Image.merge("RGB", (base_map, base_map, base_map))

    scale = 2
    scaled_map = base_rgb.resize(
        (base_rgb.width * scale, base_rgb.height * scale),
        resample=Image.Resampling.NEAREST,
    )

    left_padding = 20
    top_padding = 50
    right_padding = 380
    bottom_padding = 20
    canvas = Image.new(
        "RGB",
        (
            left_padding + scaled_map.width + right_padding,
            scaled_map.height + top_padding + bottom_padding,
        ),
        color=(236, 236, 236),
    )
    canvas.paste(scaled_map, (left_padding, top_padding))

    draw = ImageDraw.Draw(canvas)
    font = _load_font(18)
    title_font = _load_font(24)

    accent = SCENE_ACCENTS.get(scene_name, (80, 80, 80))
    draw.rectangle((0, 0, canvas.width, top_padding), fill=accent)
    draw.text((16, 16), f"{scene_name} sampled static map", fill=(255, 255, 255), font=title_font)

    ordered_categories = ["victims", "fire_sources", "transport_objects"]
    for category in ordered_categories:
        style = CATEGORY_STYLES[category]
        for index, pose in enumerate(scene_data[category], start=1):
            px, py = _world_to_pixel(pose[0], pose[1], map_info, base_map.height)
            px = px * scale + left_padding
            py = py * scale + top_padding
            _draw_marker(draw, px, py, style, f"{style['prefix']}{index}", font)

    _draw_legend(
        draw,
        scene_name=scene_name,
        map_name=os.path.basename(scene_data["map_yaml"]),
        accent=accent,
        categories=ordered_categories,
        canvas_width=canvas.width,
        font=font,
        title_font=title_font,
    )

    output_path = os.path.join(OUTPUT_DIR, f"{scene_name}_sampled_map.png")
    canvas.save(output_path)
    return output_path


def _build_overview(preview_paths: List[str]) -> str:
    previews = [Image.open(path).convert("RGB") for path in preview_paths]
    width = max(image.width for image in previews)
    height = sum(image.height for image in previews) + 60 + 12 * (len(previews) - 1)

    canvas = Image.new("RGB", (width, height), color=(245, 245, 245))
    draw = ImageDraw.Draw(canvas)
    font = _load_font(24)
    draw.rectangle((0, 0, width, 48), fill=(35, 35, 35))
    draw.text((16, 16), "RoboSAR scene sampled-map overview", fill=(255, 255, 255), font=font)

    y_offset = 60
    for image in previews:
        canvas.paste(image, (0, y_offset))
        y_offset += image.height + 12

    output_path = os.path.join(OUTPUT_DIR, "scene_sampled_map_overview.png")
    canvas.save(output_path)
    return output_path


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    config = _load_scene_config()
    preview_paths = []

    for scene_name, scene_data in config["scenes"].items():
        preview_paths.append(_build_scene_preview(scene_name, scene_data))

    overview_path = _build_overview(preview_paths)
    print("Generated previews:")
    for path in preview_paths:
        print(f"  - {path}")
    print(f"  - {overview_path}")


if __name__ == "__main__":
    main()
