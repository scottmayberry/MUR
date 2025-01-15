# Compute Module Setup Guide

## Step 1. Flash CM4 module if not already flashed.
Recommended tutorial: [How to flash the eMMC on a Raspberry Pi Compute Module 4](https://www.youtube.com/watch?v=jp_mF1RknU4)

Recommend installing the example image in the repository as it has been copied directly from a functioning MUR. If so, please ignore the following steps.

## Step 2. If using provided image, setup is complete. If not, continue to step 3.

## Step 3. Install a lite version (only command line, no wasted compute on GUI).

In the raspberry pi imager, recommended settings are below for uniform access.

**Settings:**
- Default Installation:
  - hostname: mur.local
  - username: mur
  - password: mur

- Services: enable SSH with password authentication

Test ssh by going to `ssh mur@<IPADDRESS>` once powered on and booted.

## Step 4. Service Setups
Go to the service setup section, and follow the `service_setup_guide.md`.

## END