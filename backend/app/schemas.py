"""Validated request schemas for the dashboard API."""
from typing import Literal

from pydantic import BaseModel, Field


class Ingest(BaseModel):
    kind: Literal['snapshot', 'images', 'clouds']
    data: dict


class ScenarioLoad(BaseModel):
    scenario: str = Field(min_length=1, max_length=64, pattern=r'^[A-Za-z0-9_\-]+$')


class ExperimentStart(BaseModel):
    scenario: str | None = Field(default=None, max_length=64, pattern=r'^[A-Za-z0-9_\-]+$')
    trials: int = Field(default=1, ge=1, le=50)
    seed: int = Field(default=0, ge=0, le=1_000_000)
    tag: str | None = Field(default=None, max_length=64, pattern=r'^[A-Za-z0-9_\-]+$')
