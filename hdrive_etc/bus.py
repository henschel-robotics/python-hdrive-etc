"""
HDrive EtherCAT SDK — EtherCAT Bus Manager (thin wrapper)
==========================================================

Re-exports :class:`~ethercat_master.EtherCATBus` from the generic
``ethercat_master`` package so that existing ``hdrive_etc`` imports
continue to work unchanged.

All generic EtherCAT bus logic lives in the ``ethercat_master`` package.
This module provides backward compatibility.
"""

from ethercat_master.bus import EtherCATBus, _state_name  # noqa: F401

__all__ = ["EtherCATBus"]
