#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This launcher simply start Webots."""

import os
import shutil
import sys
import tarfile
import urllib.request

from enum import Enum

import tkinter as tk

from launch.actions import ExecuteProcess
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution

from webots_ros2_core.utils import get_webots_home, get_required_webots_version, get_required_webots_version_short

InstallationMode = Enum('InstallationMode', 'automatic manual path none')


class WebotsInstallationDialog:
    """This custom dialog is displayed when Webots is not installed."""

    def __init__(self, parent):
        top = self.top = tk.Toplevel(parent)
        tk.Label(top, anchor='e', justify=tk.LEFT,
                 text='Webots %s was not found in your system.\n'
                      'Would you like to proceed with:\n'
                      ' - Automatic installation in ~/.ros\n'
                      ' - Manual installation.\n'
                      ' - Specify path of already installed version.'
                      % get_required_webots_version()
                 ).grid(row=0, columnspan=3)
        tk.Button(top, text='Automatic', command=self.automatic).grid(row=1, column=0)
        tk.Button(top, text='Manual', command=self.manual).grid(row=1, column=1)
        tk.Button(top, text='Specify path', command=self.path).grid(row=1, column=2)
        self.installationMode = InstallationMode.none

    def automatic(self):
        self.installationMode = InstallationMode.automatic
        self.top.destroy()

    def manual(self):
        self.installationMode = InstallationMode.manual
        self.top.destroy()

    def path(self):
        self.installationMode = InstallationMode.path
        self.top.destroy()


class _WebotsCommandSubstitution(Substitution):
    def __init__(self, *, world, gui, mode):
        self.__gui = gui if isinstance(gui, Substitution) else TextSubstitution(text=str(gui))
        self.__mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)
        self.__world = world if isinstance(world, Substitution) else TextSubstitution(text=world)

    def install_webots(self):
        # Remove previous archive
        installationDirectory = os.path.join(os.environ['HOME'], '.ros')
        installationPath = os.path.abspath(os.path.join(installationDirectory, 'webots'))
        archiveName = 'webots-%s-x86-64.tar.bz2' % get_required_webots_version_short()
        archivePath = os.path.join(installationDirectory, archiveName)
        if os.path.exists(archivePath):
            os.remove(archivePath)
        # Remove previous webots folder
        if os.path.exists(installationPath):
            shutil.rmtree(installationPath)
        # Get Webots archive
        print('\033[33mInstalling Webots %s, this might take some time.\033[0m' % get_required_webots_version())
        url = 'https://github.com/cyberbotics/webots/releases/download/%s/' % get_required_webots_version_short()
        urllib.request.urlretrieve(url + archiveName, archivePath)
        # Extract Webots archive
        tar = tarfile.open(archivePath, 'r:bz2')
        tar.extractall(os.path.join(installationDirectory, 'webots' + get_required_webots_version_short()))
        tar.close()
        os.remove(archivePath)
        os.environ['WEBOTS_HOME'] = os.path.join(installationPath, 'webots')

    def perform(self, context):
        webots_path = get_webots_home()
        if webots_path is None:
            if context.perform_substitution(self.__gui).lower() in ['false', '0']:
                sys.exit('Missing Webots version %s' % get_required_webots_version())
            root = tk.Tk()
            root.update()
            dialog = WebotsInstallationDialog(root)
            root.wait_window(dialog.top)
            root.destroy()
            if dialog.installationMode == InstallationMode.automatic:
                self.install_webots()
                webots_path = get_webots_home()
                if webots_path is None:
                    sys.exit('Failed to install Webots %s' % get_required_webots_version())
            elif dialog.installationMode == InstallationMode.manual:
                import webbrowser
                webbrowser.open('https://github.com/cyberbotics/webots/releases/tag/%s' % get_required_webots_version_short())
                sys.exit()
            elif dialog.installationMode == InstallationMode.path:
                from tkinter import filedialog
                answer = filedialog.askdirectory(
                    initialdir=os.getcwd(),
                    title='Please select the folder where Webots %s is installed:' % get_required_webots_version()
                )
                if isinstance(answer, str):
                    os.environ['WEBOTS_HOME'] == answer
                    webots_path = get_webots_home()
                if webots_path is None:
                    sys.exit('Missing Webots version %s' % get_required_webots_version())
            elif dialog.installationMode is None:
                sys.exit('Missing Webots version %s' % get_required_webots_version())
        # Add `webots` executable to command
        if sys.platform == 'win32':
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin')
        command = [os.path.join(webots_path, 'webots')]

        # Add `world`
        command += [context.perform_substitution(self.__world)]

        # Hide welcome window
        command += ['--batch']

        # Add parameters to hide GUI if needed
        if context.perform_substitution(self.__gui).lower() in ['false', '0']:
            command += [
                '--stdout',
                '--stderr',
                '--batch',
                '--no-sandbox',
                '--minimize'
            ]

        # Add mode
        command.append('--mode=' + context.perform_substitution(self.__mode))
        return ' '.join(command)


class WebotsLauncher(ExecuteProcess):
    def __init__(self, output='screen', world=None, gui=True, mode='realtime', **kwargs):
        command = _WebotsCommandSubstitution(
            world=world,
            gui=gui,
            mode=mode
        )

        super().__init__(
            output=output,
            cmd=[command],
            shell=True,
            **kwargs
        )
