lerobot-record \
  --robot.type=so101_follower \
  --robot.port="${FOLLOWER_PORT}" \
  --robot.id="${FOLLOWER_ID}" \
  --robot.cameras='{ front: {type: opencv, index_or_path: 1, width: 3448, height: 808, fps: 30}}' \
  --teleop.type=so101_leader \
  --teleop.port="${LEADER_PORT}" \
  --teleop.id="${LEADER_ID}" \
  --dataset.repo_id="${HF_USER}/local-test" \
  --dataset.num_episodes=5 \
  --dataset.push_to_hub=false \
  --dataset.single_task="Grab the red cylinder" \
  --dataset.video_encoding_batch_size=5 \
  --display_data=true

lerobot-record \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_PORT \
    --robot.id=$FOLLOWER_ID \
    --teleop.type=so101_leader \
    --teleop.port=$LEADER_PORT \
    --teleop.id=$LEADER_ID \
    --robot.cameras='{ front: {type: opencv, index_or_path: $FOLLOWER_CAMERA_INDEX, width: 3448, height: 808, fps: 30}}' \
    --display_data=true \
    --dataset.push_to_hub=false \
    --dataset.single_task="Grab the red cylinder" \
    --dataset.repo_id="local/data-collection" \
    --dataset.num_episodes=5 \
    --dataset.video_encoding_batch_size=5




# Replay the data collection
lerobot-replay \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_PORT \
    --robot.id=$FOLLOWER_ID \
    --dataset.repo_id="local/dance" \
    --dataset.episode=0 


lerobot-train \
  --dataset.repo_id="local/data-collection" \
  --policy.type=act \
  --output_dir=outputs/train/act_so101_test \
  --job_name=act_so101_test \
  --policy.device=mps \
  --wandb.enable=true \
  --policy.repo_id="local/my_policy"
  --batch



rafaelfelix@Mac r2b_safety_lab % lerobot-train \
  --dataset.repo_id="local/data-collection" \
  --policy.type=act \
  --output_dir=outputs/train/act_so101_test \
  --job_name=act_so101_test \
  --policy.device=mps \
  --wandb.enable=true \
  --policy.repo_id="local/my_policy" \
  --batch_size=2
  
  
  --help
usage: lerobot-train [-h] [--config_path str] [--dataset str] [--dataset.repo_id str] [--dataset.root [str]] [--dataset.episodes [List]] [--image_transforms str] [--dataset.image_transforms.enable bool] [--dataset.image_transforms.max_num_transforms int]
                     [--dataset.image_transforms.random_order bool] [--dataset.image_transforms.tfs Dict] [--dataset.revision [str]] [--dataset.use_imagenet_stats bool] [--dataset.video_backend str] [--dataset.streaming bool] [--env str]
                     [--env.type {aloha,pusht,gym_manipulator,libero,metaworld}] [--env.visualization_width int] [--env.visualization_height int] [--robot str] [--env.robot.type {}] [--teleop str] [--env.teleop.type {}] [--processor str]
                     [--env.processor.control_mode str] [--observation str] [--env.processor.observation.add_joint_velocity_to_observation bool] [--env.processor.observation.add_current_to_observation bool]
                     [--env.processor.observation.display_cameras bool] [--image_preprocessing str] [--env.processor.image_preprocessing.crop_params_dict [Dict]] [--env.processor.image_preprocessing.resize_size [int int]] [--gripper str]
                     [--env.processor.gripper.use_gripper bool] [--env.processor.gripper.gripper_penalty float] [--reset str] [--env.processor.reset.fixed_reset_joint_positions [Any]] [--env.processor.reset.reset_time_s float]
                     [--env.processor.reset.control_time_s float] [--env.processor.reset.terminate_on_success bool] [--inverse_kinematics str] [--env.processor.inverse_kinematics.urdf_path [str]]
                     [--env.processor.inverse_kinematics.target_frame_name [str]] [--env.processor.inverse_kinematics.end_effector_bounds [Dict]] [--env.processor.inverse_kinematics.end_effector_step_sizes [Dict]] [--reward_classifier str]
                     [--env.processor.reward_classifier.pretrained_path [str]] [--env.processor.reward_classifier.success_threshold float] [--env.processor.reward_classifier.success_reward float] [--env.processor.max_gripper_pos [float]] [--env.name str]
                     [--env.camera_name str] [--env.init_states bool] [--env.camera_name_mapping [Dict]] [--env.observation_height int] [--env.observation_width int] [--env.task str] [--env.fps int] [--env.features Dict] [--env.features_map Dict]
                     [--env.max_parallel_tasks int] [--env.disable_env_checker bool] [--env.episode_length int] [--env.obs_type str] [--env.render_mode str] [--env.multitask_eval bool] [--policy str]
                     [--policy.type {act,diffusion,groot,pi0,pi05,smolvla,tdmpc,vqbet,sac,reward_classifier}] [--policy.replace_final_stride_with_dilation int] [--policy.pre_norm bool] [--policy.dim_model int] [--policy.n_heads int]
                     [--policy.dim_feedforward int] [--policy.feedforward_activation str] [--policy.n_encoder_layers int] [--policy.n_decoder_layers int] [--policy.use_vae bool] [--policy.n_vae_encoder_layers int]
                     [--policy.temporal_ensemble_coeff [float]] [--policy.kl_weight float] [--policy.optimizer_lr_backbone float] [--policy.drop_n_last_frames int] [--policy.use_separate_rgb_encoder_per_camera bool] [--policy.down_dims int [int, ...]]
                     [--policy.kernel_size int] [--policy.n_groups int] [--policy.diffusion_step_embed_dim int] [--policy.use_film_scale_modulation bool] [--policy.noise_scheduler_type str] [--policy.num_train_timesteps int] [--policy.beta_schedule str]
                     [--policy.beta_start float] [--policy.beta_end float] [--policy.prediction_type str] [--policy.clip_sample bool] [--policy.clip_sample_range float] [--policy.do_mask_loss_for_padding bool] [--policy.scheduler_name str]
                     [--policy.image_size int int] [--policy.base_model_path str] [--policy.tokenizer_assets_repo str] [--policy.embodiment_tag str] [--policy.tune_llm bool] [--policy.tune_visual bool] [--policy.tune_projector bool]
                     [--policy.tune_diffusion_model bool] [--policy.lora_rank int] [--policy.lora_alpha int] [--policy.lora_dropout float] [--policy.lora_full_model bool] [--policy.warmup_ratio float] [--policy.use_bf16 bool] [--policy.video_backend str]
                     [--policy.balance_dataset_weights bool] [--policy.balance_trajectory_weights bool] [--policy.dataset_paths [List]] [--policy.output_dir str] [--policy.save_steps int] [--policy.max_steps int] [--policy.batch_size int]
                     [--policy.dataloader_num_workers int] [--policy.report_to str] [--policy.resume bool] [--policy.paligemma_variant str] [--policy.action_expert_variant str] [--policy.dtype str] [--policy.num_inference_steps int]
                     [--policy.time_sampling_beta_alpha float] [--policy.time_sampling_beta_beta float] [--policy.time_sampling_scale float] [--policy.time_sampling_offset float] [--policy.image_resolution int int] [--policy.gradient_checkpointing bool]
                     [--policy.compile_model bool] [--policy.compile_mode str] [--policy.chunk_size int] [--policy.max_state_dim int] [--policy.max_action_dim int] [--policy.resize_imgs_with_padding int int] [--policy.empty_cameras int]
                     [--policy.adapt_to_pi_aloha bool] [--policy.use_delta_joint_actions_aloha bool] [--policy.tokenizer_max_length int] [--policy.num_steps int] [--policy.use_cache bool] [--policy.train_expert_only bool] [--policy.train_state_proj bool]
                     [--policy.optimizer_grad_clip_norm float] [--policy.scheduler_decay_steps int] [--policy.scheduler_decay_lr float] [--policy.vlm_model_name str] [--policy.load_vlm_weights bool] [--policy.add_image_special_tokens bool]
                     [--policy.attention_mode str] [--policy.prefix_length int] [--policy.pad_language_to str] [--policy.num_expert_layers int] [--policy.num_vlm_layers int] [--policy.self_attn_every_n_layers int] [--policy.expert_width_multiplier float]
                     [--policy.min_period float] [--policy.max_period float] [--rtc_config str] [--policy.rtc_config.enabled bool] [--policy.rtc_config.prefix_attention_schedule RTCAttentionSchedule] [--policy.rtc_config.max_guidance_weight float]
                     [--policy.rtc_config.execution_horizon int] [--policy.rtc_config.debug bool] [--policy.rtc_config.debug_maxlen int] [--policy.n_action_repeats int] [--policy.horizon int] [--policy.n_action_steps int] [--policy.q_ensemble_size int]
                     [--policy.mlp_dim int] [--policy.use_mpc bool] [--policy.cem_iterations int] [--policy.max_std float] [--policy.min_std float] [--policy.n_gaussian_samples int] [--policy.n_pi_samples int]
                     [--policy.uncertainty_regularizer_coeff float] [--policy.n_elites int] [--policy.elite_weighting_temperature float] [--policy.gaussian_mean_momentum float] [--policy.max_random_shift_ratio float] [--policy.reward_coeff float]
                     [--policy.expectile_weight float] [--policy.value_coeff float] [--policy.consistency_coeff float] [--policy.advantage_scaling float] [--policy.pi_coeff float] [--policy.temporal_decay_coeff float]
                     [--policy.target_model_momentum float] [--policy.n_action_pred_token int] [--policy.action_chunk_size int] [--policy.vision_backbone str] [--policy.crop_shape [int int]] [--policy.crop_is_random bool]
                     [--policy.pretrained_backbone_weights [str]] [--policy.use_group_norm bool] [--policy.spatial_softmax_num_keypoints int] [--policy.n_vqvae_training_steps int] [--policy.vqvae_n_embed int] [--policy.vqvae_embedding_dim int]
                     [--policy.vqvae_enc_hidden_dim int] [--policy.gpt_block_size int] [--policy.gpt_input_dim int] [--policy.gpt_output_dim int] [--policy.gpt_n_layer int] [--policy.gpt_n_head int] [--policy.gpt_hidden_dim int] [--policy.dropout float]
                     [--policy.offset_loss_weight float] [--policy.primary_code_loss_weight float] [--policy.secondary_code_loss_weight float] [--policy.bet_softmax_temperature float] [--policy.sequentially_select bool] [--policy.optimizer_lr float]
                     [--policy.optimizer_betas Any] [--policy.optimizer_eps float] [--policy.optimizer_weight_decay float] [--policy.optimizer_vqvae_lr float] [--policy.optimizer_vqvae_weight_decay float] [--policy.scheduler_warmup_steps int]
                     [--policy.dataset_stats [Dict]] [--policy.storage_device str] [--policy.vision_encoder_name [str]] [--policy.freeze_vision_encoder bool] [--policy.image_encoder_hidden_dim int] [--policy.shared_encoder bool]
                     [--policy.num_discrete_actions [int]] [--policy.online_steps int] [--policy.online_buffer_capacity int] [--policy.offline_buffer_capacity int] [--policy.async_prefetch bool] [--policy.online_step_before_learning int]
                     [--policy.policy_update_freq int] [--policy.discount float] [--policy.temperature_init float] [--policy.num_critics int] [--policy.num_subsample_critics [int]] [--policy.critic_lr float] [--policy.actor_lr float]
                     [--policy.temperature_lr float] [--policy.critic_target_update_weight float] [--policy.utd_ratio int] [--policy.state_encoder_hidden_dim int] [--policy.target_entropy [float]] [--policy.use_backup_entropy bool]
                     [--critic_network_kwargs str] [--policy.critic_network_kwargs.hidden_dims List] [--policy.critic_network_kwargs.activate_final bool] [--policy.critic_network_kwargs.final_activation [str]] [--actor_network_kwargs str]
                     [--policy.actor_network_kwargs.hidden_dims List] [--policy.actor_network_kwargs.activate_final bool] [--policy_kwargs str] [--policy.policy_kwargs.use_tanh_squash bool] [--policy.policy_kwargs.std_min float]
                     [--policy.policy_kwargs.std_max float] [--policy.policy_kwargs.init_final float] [--discrete_critic_network_kwargs str] [--policy.discrete_critic_network_kwargs.hidden_dims List]
                     [--policy.discrete_critic_network_kwargs.activate_final bool] [--policy.discrete_critic_network_kwargs.final_activation [str]] [--actor_learner_config str] [--policy.actor_learner_config.learner_host str]
                     [--policy.actor_learner_config.learner_port int] [--policy.actor_learner_config.policy_parameters_push_frequency int] [--policy.actor_learner_config.queue_get_timeout float] [--concurrency str] [--policy.concurrency.actor str]
                     [--policy.concurrency.learner str] [--policy.use_torch_compile bool] [--policy.n_obs_steps int] [--policy.input_features Dict] [--policy.output_features Dict] [--policy.device str] [--policy.use_amp bool] [--policy.push_to_hub bool]
                     [--policy.repo_id [str]] [--policy.private [bool]] [--policy.tags [List]] [--policy.license [str]] [--policy.pretrained_path [Path]] [--policy.name str] [--policy.num_classes int] [--policy.hidden_dim int] [--policy.latent_dim int]
                     [--policy.image_embedding_pooling_dim int] [--policy.dropout_rate float] [--policy.model_name str] [--policy.model_type str] [--policy.num_cameras int] [--policy.learning_rate float] [--policy.weight_decay float]
                     [--policy.grad_clip_norm float] [--policy.normalization_mapping Dict] [--output_dir [Path]] [--job_name [str]] [--resume bool] [--seed [int]] [--num_workers int] [--batch_size int] [--steps int] [--eval_freq int] [--log_freq int]
                     [--save_checkpoint bool] [--save_freq int] [--use_policy_training_preset bool] [--optimizer str] [--optimizer.type {adam,adamw,sgd,multi_adam}] [--optimizer.betas float float] [--optimizer.eps float] [--optimizer.momentum float]
                     [--optimizer.dampening float] [--optimizer.nesterov bool] [--optimizer.lr float] [--optimizer.weight_decay float] [--optimizer.grad_clip_norm float] [--optimizer.optimizer_groups Dict] [--scheduler str]
                     [--scheduler.type {diffuser,vqbet,cosine_decay_with_warmup}] [--scheduler.name str] [--scheduler.num_vqvae_training_steps int] [--scheduler.num_cycles float] [--scheduler.num_warmup_steps int] [--scheduler.num_decay_steps int]
                     [--scheduler.peak_lr float] [--scheduler.decay_lr float] [--eval str] [--eval.n_episodes int] [--eval.batch_size int] [--eval.use_async_envs bool] [--wandb str] [--wandb.enable bool] [--wandb.disable_artifact bool]
                     [--wandb.project str] [--wandb.entity [str]] [--wandb.notes [str]] [--wandb.run_id [str]] [--wandb.mode [str]] [--rename_map Dict]
lerobot-train: error: argument --output_dir: invalid pathlib.Path | None value: 'outputs/train/act_so101_test'