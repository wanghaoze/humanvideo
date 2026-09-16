#!/usr/bin/env python3
"""CPU API smoke check with a tiny random model; no pretrained downloads."""
import argparse
from pathlib import Path
import sys

import torch
import transformers
from transformers import DINOv3ViTConfig, DINOv3ViTModel

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1] / 'third_party/TRELLIS.2')
args = parser.parse_args()
print('Transformers:', transformers.__version__)
assert transformers.__version__ == '4.57.3', 'Run: python -m pip install transformers==4.57.3'
sys.path.insert(0, str(args.repo.resolve()))
from trellis2.modules.image_feature_extractor import DinoV3FeatureExtractor

config = DINOv3ViTConfig(image_size=32, patch_size=16, hidden_size=64,
                       intermediate_size=128, num_hidden_layers=1,
                       num_attention_heads=4, num_register_tokens=4)
model = DINOv3ViTModel(config).eval()
assert hasattr(model, 'layer'), 'DINOv3 .layer API unavailable'
extractor = DinoV3FeatureExtractor.__new__(DinoV3FeatureExtractor)
extractor.model = model
with torch.inference_mode():
    features = extractor.extract_features(torch.randn(1, 3, 32, 32))
assert features.shape == (1, 9, 64), features.shape
assert torch.isfinite(features).all().item()
print('PASS: actual TRELLIS.2 DINOv3 extractor on a tiny CPU model')
