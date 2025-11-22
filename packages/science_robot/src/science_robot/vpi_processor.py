"""
VPI (Vision Programming Interface) module for GPU-accelerated image processing
Provides GPU-accelerated operations for better performance on Jetson Nano
"""
import numpy as np
import logging

logger = logging.getLogger(__name__)

try:
    import vpi
    # Verify VPI is actually functional, not just importable
    # Check if Backend attribute exists
    if hasattr(vpi, 'Backend'):
        try:
            # Try to access a basic VPI backend to verify it's working
            _ = vpi.Backend.CUDA
            VPI_AVAILABLE = True
        except (AttributeError, Exception):
            # VPI module exists but Backend may not be functional
            VPI_AVAILABLE = False
    else:
        # VPI module exists but doesn't have Backend attribute
        # This might be an older version or incomplete installation
        VPI_AVAILABLE = False
except ImportError:
    VPI_AVAILABLE = False
    # Don't log here - logger may not be configured yet
    pass


class VPIProcessor:
    """GPU-accelerated image processing using NVIDIA VPI"""
    
    def __init__(self, backend='CUDA'):
        """
        Initialize VPI processor
        
        Args:
            backend: VPI backend ('CUDA', 'CPU', 'VIC', 'PVA', or 'OFA')
                     Note: 'GPU' is accepted as alias for 'CUDA'
        """
        self.available = VPI_AVAILABLE
        self.backend = backend
        
        if self.available and hasattr(vpi, 'Backend'):
            # Map user-friendly backend names to VPI backend enums
            # VPI uses CUDA for GPU acceleration, not 'GPU'
            backend_map = {
                'CUDA': vpi.Backend.CUDA,  # GPU acceleration
                'GPU': vpi.Backend.CUDA,   # Alias for CUDA (backward compatibility)
                'CPU': vpi.Backend.CPU,
                'VIC': vpi.Backend.VIC,    # Video Image Compositor
                'PVA': vpi.Backend.PVA,    # Programmable Vision Accelerator
                'OFA': vpi.Backend.OFA     # Optical Flow Accelerator
            }
            
            # Normalize backend name (handle case variations)
            backend_upper = backend.upper()
            if backend_upper not in backend_map:
                logger.warning(f"Unknown VPI backend '{backend}', defaulting to CUDA")
                backend_upper = 'CUDA'
            
            # Use getattr to safely access backend attributes
            backend_enum = None
            if hasattr(vpi, 'Backend'):
                for attempt_name in [backend_upper, 'CUDA', 'CPU']:
                    backend_attr = getattr(vpi.Backend, attempt_name, None)
                    if backend_attr is not None:
                        backend_enum = backend_attr
                        if attempt_name != backend_upper:
                            logger.warning(f"Requested backend '{backend_upper}' not available, using '{attempt_name}'")
                        break
            
            if backend_enum is None:
                logger.warning("No VPI backends available. VPI installation may be incomplete. Using CPU fallback.")
                self.vpi_backend = None
                self.available = False
            else:
                self.vpi_backend = backend_enum
                # Log the actual backend name (not alias)
                actual_backend = 'CUDA' if backend_upper == 'GPU' else backend_upper
                logger.info(f"VPI processor initialized with {actual_backend} backend")
        else:
            # Provide more helpful diagnostic information
            import sys
            python_paths = '\n'.join(sys.path[:5])  # Show first 5 paths
            logger.warning("VPI processor unavailable - using CPU fallback.")
            logger.debug(f"Python path: {python_paths}")
            logger.debug("VPI is typically part of NVIDIA JetPack SDK on Jetson devices.")
            logger.debug("If running on Jetson, ensure JetPack is installed and VPI libraries are accessible.")
            self.vpi_backend = None
    
    def resize_gpu(self, image, target_width, target_height):
        """
        Resize image using GPU acceleration
        
        Args:
            image: Input image (numpy array)
            target_width: Target width
            target_height: Target height
            
        Returns:
            Resized image (numpy array)
        """
        if not self.available:
            # Fallback to OpenCV CPU resize
            import cv2
            return cv2.resize(image, (target_width, target_height))
        
        try:
            # Convert numpy array to VPI image
            vpi_img = vpi.asimage(image)
            
            # Ensure backend is set (should not happen, but safety check)
            if self.vpi_backend is None:
                raise AttributeError("VPI backend not initialized")
            
            # VPI Image object has a rescale method for resizing
            # Try different possible API patterns
            resized = None
            try:
                # Method 1: Use rescale method directly (most direct)
                # rescale requires backend parameter (mandatory in VPI)
                # Some formats may not be supported by all backends
                if hasattr(vpi_img, 'rescale'):
                    # Try with target size tuple and requested backend
                    try:
                        resized = vpi_img.rescale((target_width, target_height), 
                                                 interp=vpi.Interp.LINEAR, 
                                                 backend=self.vpi_backend)
                    except ValueError as format_error:
                        # Format may not be supported by this backend, try CPU
                        if hasattr(vpi, 'Backend') and 'format' in str(format_error).lower() and self.vpi_backend != vpi.Backend.CPU:
                            logger.debug(f"Format not supported by {self.backend} backend, trying CPU")
                            resized = vpi_img.rescale((target_width, target_height), 
                                                     interp=vpi.Interp.LINEAR, 
                                                     backend=vpi.Backend.CPU)
                        else:
                            raise
                    except (TypeError, AttributeError):
                        # Maybe it takes scale factors instead
                        scale_x = target_width / vpi_img.width
                        scale_y = target_height / vpi_img.height
                        try:
                            resized = vpi_img.rescale((scale_x, scale_y), 
                                                     interp=vpi.Interp.LINEAR, 
                                                     backend=self.vpi_backend)
                        except ValueError:
                            # Try CPU backend if format error
                            if hasattr(vpi, 'Backend'):
                                resized = vpi_img.rescale((scale_x, scale_y), 
                                                         interp=vpi.Interp.LINEAR, 
                                                         backend=vpi.Backend.CPU)
                            else:
                                raise
                else:
                    raise AttributeError("rescale method not found")
            except (AttributeError, TypeError) as e1:
                try:
                    # Method 2: Try Conversion with HomographyTransform2D (fallback)
                    scale_x = target_width / vpi_img.width
                    scale_y = target_height / vpi_img.height
                    scale_transform = vpi.HomographyTransform2D([scale_x, 0, 0, 
                                                                 0, scale_y, 0, 
                                                                 0, 0, 1])
                    output_img = vpi.Image((target_width, target_height), vpi_img.format)
                    conv = vpi.Conversion(vpi_img, output_img, scale_transform, 
                                         interp=vpi.Interp.LINEAR, backend=self.vpi_backend)
                    vpi.execute(conv)
                    resized = output_img
                except Exception as e2:
                    logger.debug(f"VPI resize attempts failed: {e1}, {e2}")
                    raise AttributeError("VPI resize API not available")
            
            # Convert back to numpy
            return resized.cpu()
        except Exception as e:
            logger.warning(f"VPI resize error: {e}, falling back to CPU")
            import cv2
            return cv2.resize(image, (target_width, target_height))
    
    def gaussian_blur_gpu(self, image, kernel_size=5, sigma=1.0):
        """
        Apply Gaussian blur using GPU acceleration
        
        Args:
            image: Input image (numpy array)
            kernel_size: Blur kernel size (must be odd)
            sigma: Gaussian sigma value
            
        Returns:
            Blurred image (numpy array)
        """
        if not self.available:
            # Fallback to OpenCV CPU blur
            import cv2
            return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
        
        try:
            vpi_img = vpi.asimage(image)
            # VPI Image object has gaussian_filter method
            blurred = None
            try:
                # Use gaussian_filter method on Image object
                if hasattr(vpi_img, 'gaussian_filter'):
                    blurred = vpi_img.gaussian_filter(kernel_size, sigma, backend=self.vpi_backend)
                else:
                    raise AttributeError("gaussian_filter method not found")
            except (AttributeError, TypeError) as e1:
                try:
                    # Fallback: Try if Gaussian is a top-level function
                    blurred = vpi.Gaussian(vpi_img, kernel_size, sigma, backend=self.vpi_backend)
                except (AttributeError, TypeError) as e2:
                    logger.debug(f"VPI Gaussian attempts failed: {e1}, {e2}")
                    raise AttributeError("VPI Gaussian not available")
            return blurred.cpu()
        except Exception as e:
            logger.warning(f"VPI blur error: {e}, falling back to CPU")
            import cv2
            return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
    
    def convert_color_gpu(self, image, conversion_code):
        """
        Convert color space using GPU acceleration
        
        Args:
            image: Input image (numpy array)
            conversion_code: OpenCV color conversion code (e.g., cv2.COLOR_BGR2RGB)
            
        Returns:
            Converted image (numpy array)
        """
        if not self.available:
            # Fallback to OpenCV CPU conversion
            import cv2
            return cv2.cvtColor(image, conversion_code)
        
        try:
            vpi_img = vpi.asimage(image)
            
            # Map OpenCV conversion codes to VPI conversions
            # Note: VPI has limited conversion support
            # For BGR2RGB, we'll use CPU fallback
            import cv2
            return cv2.cvtColor(image, conversion_code)
        except Exception as e:
            logger.warning(f"VPI color conversion error: {e}, falling back to CPU")
            import cv2
            return cv2.cvtColor(image, conversion_code)
    
    def is_available(self):
        """Check if VPI is available"""
        return self.available
    
    def get_available_backends(self):
        """
        Get list of available VPI backends
        
        Returns:
            List of available backend names, or empty list if VPI unavailable
        """
        if not self.available or not hasattr(vpi, 'Backend'):
            return []
        
        available = []
        backend_names = ['CPU', 'CUDA', 'VIC', 'PVA', 'OFA']
        
        for name in backend_names:
            try:
                backend_attr = getattr(vpi.Backend, name, None)
                if backend_attr is not None:
                    available.append(name)
            except AttributeError:
                pass
        
        return available

