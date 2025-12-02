---
sidebar_position: 6
---

# Vision-Language-Action (VLA)

## What is VLA?

**Vision-Language-Action (VLA)** models are multimodal AI systems that combine:
- **Vision**: Understanding visual scenes (cameras, depth sensors)
- **Language**: Processing natural language commands
- **Action**: Generating robot control policies

:::tip Key Insight
VLAs enable robots to follow natural language instructions while perceiving and interacting with the physical world.
:::

## Architecture Overview

```
Text Command → Language Encoder (BERT/GPT)
                      ↓
Image Input   → Vision Encoder (ViT/ResNet) → Fusion Network → Policy Network → Actions
                      ↓
             Multimodal Transformer
```

## Why VLA Matters

Traditional robots require explicit programming for each task. VLAs enable:
- ✅ **Generalization**: Learn from demonstrations, apply to new scenarios
- ✅ **Natural interaction**: Understand human instructions
- ✅ **Few-shot learning**: Adapt quickly with minimal examples
- ✅ **End-to-end**: Map perception directly to actions

## Key Models

### 1. **RT-1 (Robotics Transformer 1)**
*Google's breakthrough VLA model*

- **Architecture**: Vision Transformer (ViT) + FiLM (Feature-wise Linear Modulation)
- **Training**: 130k robot demonstrations across 700+ tasks
- **Performance**: 97% success on seen tasks, 76% on novel tasks

```python
# Conceptual RT-1 architecture
class RT1(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision_encoder = ViT(image_size=300)
        self.language_encoder = BERTEncoder()
        self.policy_head = TransformerDecoder()
    
    def forward(self, images, text):
        # Encode image
        img_features = self.vision_encoder(images)
        
        # Encode text
        text_features = self.language_encoder(text)
        
        # Fuse modalities with FiLM
        fused = self.film_layer(img_features, text_features)
        
        # Generate actions (x, y, z, gripper)
        actions = self.policy_head(fused)
        return actions
```

### 2. **RT-2 (Robotics Transformer 2)**
*Scaling VLAs with web data*

- Initialized from **PaLM-E** (visual-language model)
- Pre-trained on internet images + text
- Fine-tuned on robot data
- Better generalization to unseen objects and tasks

### 3. **PaLM-E**
*Embodied multimodal language model*

- **Size**: 562B parameters
- **Capabilities**: VQA (Visual Question Answering), planning, control
- **Integration**: Direct robot control from language

### 4. **OpenVLA**
*Open-source VLA (from Berkeley)*

- 7B parameter model
- Trained on Open X-Embodiment dataset
- Compatible with multiple robot platforms

## Training a VLA

### Data Collection

```python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class DataCollector:
    def __init__(self):
        self.images = []
        self.actions = []
        self.language_goals = []
    
    def collect_demo(self, language_instruction):
        # Record human demonstration
        while not task_complete:
            # Capture image
            image = self.camera.get_frame()
            
            # Capture action (from teleoperation)
            action = self.get_current_action()
            
            # Store
            self.images.append(image)
            self.actions.append(action)
            self.language_goals.append(language_instruction)
        
        # Save to dataset
        self.save_trajectory()
```

### Model Training

```python
import torch
import torch.nn as nn
from transformers import AutoModel

class VLAPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        # Vision encoder
        self.vision = AutoModel.from_pretrained("google/vit-base-patch16-224")
        
        # Language encoder
        self.language = AutoModel.from_pretrained("bert-base-uncased")
        
        # Fusion
        self.fusion = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model=768, nhead=8),
            num_layers=6
        )
        
        # Action head (x, y, z, gripper_open)
        self.action_head = nn.Linear(768, 4)
    
    def forward(self, images, text_tokens):
        # Encode vision
        vision_emb = self.vision(pixel_values=images).last_hidden_state
        
        # Encode language
        lang_emb = self.language(input_ids=text_tokens).last_hidden_state
        
        # Concatenate and fuse
        combined = torch.cat([vision_emb, lang_emb], dim=1)
        fused = self.fusion(combined)
        
        # Predict action
        action = self.action_head(fused[:, 0, :])  # Use CLS token
        return action

# Training loop
model = VLAPolicy()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

for epoch in range(100):
    for batch in dataloader:
        images, text, actions = batch
        
        # Forward pass
        predicted_actions = model(images, text)
        
        # Loss (MSE for continuous actions)
        loss = nn.MSELoss()(predicted_actions, actions)
        
        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

## Deployment Example

### ROS 2 VLA Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import torch
from cv_bridge import CvBridge

class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')
        
        # Load model
        self.model = VLAPolicy()
        self.model.load_state_dict(torch.load('vla_model.pth'))
        self.model.eval()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10)
        
        # Publisher
        self.action_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.current_image = None
        self.current_command = ""
    
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg)
    
    def command_callback(self, msg):
        self.current_command = msg.data
        self.execute_command()
    
    def execute_command(self):
        if self.current_image is None:
            return
        
        # Preprocess
        image_tensor = self.preprocess_image(self.current_image)
        text_tensor = self.tokenize_text(self.current_command)
        
        # Inference
        with torch.no_grad():
            action = self.model(image_tensor, text_tensor)
        
        # Publish action
        cmd_vel = Twist()
        cmd_vel.linear.x = float(action[0])
        cmd_vel.angular.z = float(action[1])
        self.action_pub.publish(cmd_vel)
        
        self.get_logger().info(f'Executed: {self.current_command}')
```

## Real-World Applications

### 1. **Household Robots**
"Pick up the red cup from the table"

### 2. **Warehouse Automation**
"Move the box from Shelf A3 to Packing Station 2"

### 3. **Assistive Robotics**
"Help me put on my jacket"

### 4. **Agricultural Robots**
"Harvest the ripe tomatoes"

## Challenges & Solutions

| Challenge | Solution |
|-----------|----------|
| **Data Scarcity** | Use simulation + domain randomization |
| **Generalization** | Pre-train on internet data (RT-2 approach) |
| **Safety** | Add constrained action spaces + human oversight |
| **Real-time Performance** | Optimize with TensorRT, quantization |

## Advanced: Multimodal Chain-of-Thought

Enhance VLAs with reasoning:

```python
class VLAWithCoT(nn.Module):
    def __init__(self):
        super().__init__()
        self.vla_base = VLAPolicy()
        self.reasoning_llm = GPT4Vision()
    
    def forward(self, image, instruction):
        # Step 1: Generate reasoning plan
        prompt = f"Task: {instruction}\nBreak into steps:"
        plan = self.reasoning_llm(image, prompt)
        
        # Step 2: Execute each step with VLA
        for step in plan:
            action = self.vla_base(image, step)
            execute_action(action)
            image = get_new_observation()
        
        return "Task completed"
```

## Open X-Embodiment Dataset

Large-scale multi-robot dataset for training VLAs:

- **1M+ trajectories**
- **22 different robot embodiments**
- **527 skills across multiple domains**

```python
# Load Open X-Embodiment
from datasets import load_dataset

dataset = load_dataset("google/open-x-embodiment")

# Access samples
for sample in dataset['train']:
    images = sample['observation']['image']
    language = sample['language_instruction']
    actions = sample['action']
```

## Exercise

### Build a Simple VLA
1. Collect 100 demonstrations for "pick and place"
2. Train a vision-language model
3. Deploy to simulated robot in Gazebo
4. Test generalization with novel objects

### Challenge
Implement hierarchical VLA:
1. High-level planner (language model)
2. Low-level controller (VLA for primitives)
3. Evaluate on multi-step tasks

## Resources

- [RT-1 Paper](https://arxiv.org/abs/2212.06817)
- [RT-2 Paper](https://arxiv.org/abs/2307.15818)
- [Open X-Embodiment](https://robotics-transformer-x.github.io/)
- [OpenVLA GitHub](https://github.com/openvla/openvla)

---

**Learning Objectives** ✅
- [ ] Understand VLA architecture
- [ ] Train a basic vision-language-action model
- [ ] Deploy VLA with ROS 2
- [ ] Use pre-trained models (RT-2, OpenVLA)
- [ ] Evaluate generalization capabilities
