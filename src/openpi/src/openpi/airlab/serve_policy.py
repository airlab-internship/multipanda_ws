import dataclasses
import logging
import socket

import tyro

from openpi.policies import policy as _policy
from openpi.policies import policy_config as _policy_config
from openpi.serving import websocket_policy_server
from openpi.airlab import config as _config


@dataclasses.dataclass
class Checkpoint:
    """Load a policy from a trained checkpoint."""

    # Training config name: "pi05_droid_finetune".
    config: str
    # Checkpoint directory (e.g., "checkpoints/pi05_droid_finetune/pick_place/10000").
    dir: str


@dataclasses.dataclass
class Args:
    """Arguments for the serve_policy script."""


    # If provided, will be used in case the "prompt" key is not present in the data, or if the model doesn't have a default
    # prompt.
    default_prompt: str | None = None

    # Port to serve the policy on.
    port: int = 8152
    # Record the policy's behavior for debugging.
    record: bool = False

    # Explicit JAX checkpoint; no unrelated environment defaults.
    policy: Checkpoint = dataclasses.field(default_factory=lambda: Checkpoint(
        config="pi05_droid_finetune", dir=tyro.MISSING,
    ))


def create_policy(args: Args) -> _policy.Policy:
    """Create a policy from the given arguments."""
    return _policy_config.create_trained_policy(
        _config.get_config(args.policy.config), args.policy.dir, default_prompt=args.default_prompt
    )


def main(args: Args) -> None:
    policy = create_policy(args)
    policy_metadata = policy.metadata

    # Record the policy's behavior.
    if args.record:
        policy = _policy.PolicyRecorder(policy, "policy_records")

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    logging.info("Creating server (host: %s, ip: %s)", hostname, local_ip)

    server = websocket_policy_server.WebsocketPolicyServer(
        policy=policy,
        host="0.0.0.0",
        port=args.port,
        metadata=policy_metadata,
    )
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main(tyro.cli(Args))
